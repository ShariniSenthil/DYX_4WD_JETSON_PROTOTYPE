param(
    [string]$RepoRoot = "."
)

$ErrorActionPreference = "Stop"

function Fail([string]$Message) {
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

function Replace-ExactlyOnce {
    param(
        [string]$Text,
        [string]$Old,
        [string]$New,
        [string]$Label
    )
    $count = ([regex]::Matches($Text, [regex]::Escape($Old))).Count
    if ($count -ne 1) {
        throw "${Label}: expected exactly 1 match, found $count"
    }
    return $Text.Replace($Old, $New)
}

$repo = (Resolve-Path $RepoRoot).Path
Set-Location $repo

$branch = (git branch --show-current).Trim()
if ($branch -ne "feat/mission_upgrade") {
    Fail "This patch is built for feat/mission_upgrade, but current branch is '$branch'."
}

$controller = "src/rpp_controller/rpp_controller/rpp_controller_node.py"
$launch = "src/rover_bringup/launch/rover.launch.py"

foreach ($f in @($controller, $launch)) {
    if (-not (Test-Path $f)) {
        Fail "Missing required file: $f"
    }
}

$trackedChanges = git status --porcelain -- $controller $launch
if ($trackedChanges) {
    Write-Host $trackedChanges
    Fail "Target files already have local changes. Commit/stash/revert them before applying."
}

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
Copy-Item $controller "$controller.pre_smart_pivot_$timestamp"
Copy-Item $launch "$launch.pre_smart_pivot_$timestamp"

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)

# -----------------------------
# 1) RPP pivot command shaping
# -----------------------------
$text = [System.IO.File]::ReadAllText((Join-Path $repo $controller))
$text = $text.Replace("`r`n", "`n")

if ($text.Contains("def _explicit_pivot_profile_bearing(")) {
    Fail "Smart pivot profile already appears to be installed."
}

$marker = "    def _publish_legacy_native_carrier(`n"
if (-not $text.Contains($marker)) {
    Fail "Could not find _publish_legacy_native_carrier() insertion point."
}

$helper = @'
    def _explicit_pivot_profile_bearing(self, true_bearing, true_error):
        """Shape the zero-translation explicit-yaw pivot reference.

        PX4 remains the only yaw dynamics controller. RPP only shapes the
        absolute-yaw reference so a large/mid-size pivot keeps useful yaw
        authority, then smoothly converges back to the real segment bearing
        before the release threshold.

        The profile reuses existing production thresholds:
          - terminal_native_pivot_release_error: final true-yaw region
          - pivot_enter_angle: full-carrier region starts here
          - terminal_native_pivot_request_error: maximum carrier lead
        """
        true_bearing = self.normalize_angle(float(true_bearing))
        true_error = self.normalize_angle(float(true_error))
        error_abs = abs(true_error)

        release_error = max(
            1.0e-6,
            float(self.terminal_native_pivot_release_error),
        )
        full_error = max(
            release_error + 1.0e-6,
            float(self.pivot_enter_angle),
        )
        carrier_error = max(
            full_error,
            float(self.terminal_native_pivot_request_error),
        )

        if error_abs <= release_error:
            requested_error = error_abs
        elif error_abs >= full_error:
            requested_error = carrier_error
        else:
            ratio = (error_abs - release_error) / (full_error - release_error)
            ratio = max(0.0, min(1.0, ratio))
            blend = ratio * ratio * (3.0 - 2.0 * ratio)
            requested_error = error_abs + blend * (carrier_error - error_abs)

        requested_error = max(
            error_abs,
            min(carrier_error, requested_error),
        )
        turn_sign = 1.0 if true_error >= 0.0 else -1.0
        return self.normalize_angle(
            self.current_yaw + turn_sign * requested_error
        )

'@

$text = $text.Replace($marker, $helper + $marker)

$oldValidation = @'
            # Patch 5 B: true zero-translation absolute-yaw pivot.
            self.reset_speed_profiles()
'@
$newValidation = @'
            # Smart production pivot reference: zero translation is preserved.
            # PX4 still owns the complete yaw-control dynamics.
            command_bearing = self._explicit_pivot_profile_bearing(
                true_bearing,
                true_error,
            )

            self.reset_speed_profiles()
'@

$text = Replace-ExactlyOnce `
    -Text $text `
    -Old $oldValidation `
    -New $newValidation `
    -Label "pivot command profile insertion"

$oldPivotPublish = @'
            if not self.velocity_pub.publish_zero_with_yaw(
                message,
                float(true_bearing),
            ):
                raise RuntimeError(
                    "B-side zero-translation explicit-yaw pivot rejected"
                )
'@

$newPivotPublish = @'
            if not self.velocity_pub.publish_zero_with_yaw(
                message,
                float(command_bearing),
            ):
                raise RuntimeError(
                    "B-side zero-translation explicit-yaw pivot rejected"
                )
'@

$text = Replace-ExactlyOnce `
    -Text $text `
    -Old $oldPivotPublish `
    -New $newPivotPublish `
    -Label "scoped explicit pivot yaw target"

$oldLog = '                + f" | target_bearing={math.degrees(true_bearing):.1f}deg"'
$newLog = '                + f" | command_bearing={math.degrees(command_bearing):.1f}deg"' + "`n" +
          '                + f" | final_bearing={math.degrees(true_bearing):.1f}deg"'
$text = Replace-ExactlyOnce `
    -Text $text `
    -Old $oldLog `
    -New $newLog `
    -Label "pivot log fields"

[System.IO.File]::WriteAllText(
    (Join-Path $repo $controller),
    $text,
    $utf8NoBom
)

# ----------------------------------------
# 2) Production launch calibration changes
# ----------------------------------------
$launchText = [System.IO.File]::ReadAllText((Join-Path $repo $launch))
$launchText = $launchText.Replace("`r`n", "`n")

# Steering: faster response, but keep authority below the old unstable 30-deg cone.
$launchText = Replace-ExactlyOnce `
    -Text $launchText `
    -Old '"precision_lookahead_time_s": 0.90' `
    -New '"precision_lookahead_time_s": 0.65' `
    -Label "precision lookahead"

$launchText = Replace-ExactlyOnce `
    -Text $launchText `
    -Old '"precision_moving_bearing_cone_deg": 15.0' `
    -New '"precision_moving_bearing_cone_deg": 18.0' `
    -Label "moving steering cone"

# Pivot: hold the explicit final-yaw authority closer to the true line before release.
$launchText = Replace-ExactlyOnce `
    -Text $launchText `
    -Old '"terminal_native_pivot_release_error_deg": 4.0' `
    -New '"terminal_native_pivot_release_error_deg": 2.0' `
    -Label "pivot release heading"

# Keep these intentionally unchanged:
#   precision_pivot_enabled = False
#   precision_pivot_stop_speed_tolerance_mps = 0.060
#   precision_explicit_yaw_rate_limit_degps = 25.0
#   legacy_pivot_post_settle_hold_sec = 0.20
# The active legacy alignment path is retained because the dormant precision
# FSM's 30-mm anchor-drift recenter rule conflicts with measured pivot motion.

[System.IO.File]::WriteAllText(
    (Join-Path $repo $launch),
    $launchText,
    $utf8NoBom
)

Write-Host ""
Write-Host "SMART RPP PIVOT/STEERING PATCH APPLIED" -ForegroundColor Green
Write-Host "Branch: $branch"
Write-Host ""
Write-Host "Changed:"
Write-Host "  - Active legacy explicit-yaw pivot now uses a smooth carrier profile"
Write-Host "  - Pivot release heading: 4.0 deg -> 2.0 deg"
Write-Host "  - Precision lookahead: 0.90 s -> 0.65 s"
Write-Host "  - Moving bearing cone: 15 deg -> 18 deg"
Write-Host "  - precision_pivot_enabled remains FALSE"
Write-Host "  - PX4/RoboClaw parameters untouched"
Write-Host ""
Write-Host "Backups:"
Write-Host "  $controller.pre_smart_pivot_$timestamp"
Write-Host "  $launch.pre_smart_pivot_$timestamp"
Write-Host ""
Write-Host "Now run:"
Write-Host "  git diff --check"
Write-Host "  git diff -- $controller"
Write-Host "  git diff -- $launch"
