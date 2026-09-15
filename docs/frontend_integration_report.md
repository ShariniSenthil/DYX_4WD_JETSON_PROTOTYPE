# Frontend Integration Hardening Report

This document specifies the exact frontend changes required to harden the frontend ↔ backend integration. The backend components have already been implemented to support these changes.

## 1. Mission Report Export (Fix Data Source)

Currently, the export generation can rely on a runtime fallback snapshot rather than the true final canonical report.

- **`src/adapters/backendMissionReportAdapter.ts`**:
  - Implement a new function `projectCanonicalReportForExport(report: CanonicalMissionReport | null): Record<number, any>` to map the canonical backend report structure into the exact prop shape expected by `MissionReportExport.tsx`. Convert `overall_accuracy_mm` to `position_error_cm` (divide by 10) for compatibility.
- **`src/screens/MissionReportScreen.tsx` (approx line 4981)**:
  - Modify the export modal trigger inside `MissionTableToolbarActions` to fetch and await the final `getMissionReport()` for the `mission_run_id` before opening the modal or feeding the data.
- **`src/components/missionreport/MissionCompletionDialog.tsx` (approx line 123)**:
  - Apply the same fetch/await swap before feeding data to `MissionReportExport`.
- **`src/components/missionreport/MissionReportExport.tsx`**:
  - Expose an async data fetch prop (e.g., `fetchExportData`) so it can trigger the fetch before launching the modal.

## 2. Mission Execution: RESUME Staged Progress & Timeout Clarity

The backend now tracks `resume_stage` during the settle→offboard→arm sequence (similarly to `start_stage`). Additionally, genuine timeouts return HTTP 504 with `outcome: "UNKNOWN"`.

- **`src/context/SocketEventCoordinator.tsx`**:
  - Capture `state`, `state_lower`, `start_stage`, `start_failed_stage`, and the newly added `resume_stage` from the `mission_status` event and expose them via context.
- **`src/services/apiError.ts`**:
  - Ensure the parser extracts `outcome`, `retry_safe`, and `mission` from the JSON `detail` body of 409/504 responses and assigns them to the `ApiError` instance.
- **`src/components/missionreport/MissionControlCard.tsx`**:
  - **Start Phase**: Rewire `startPhaseLabel` (approx lines 159-168) to key off the real `start_stage` value instead of the dead `mission.status` path.
  - **Resume Phase**: Add a `resumePhaseLabel` reading `resume_stage` and wire it into the Resume button using the same pattern as Start's disable/label logic (approx lines 474-511).
  - **Error Handling**: Branch on `outcome === "UNKNOWN"` in the catch blocks for `handleResume` and `handleStart`, providing a specific toast like "Resume result unknown — check mission status before retrying" instead of the generic failure toast.

## 3. Telemetry/Status: Unrendered Fields, Per-Waypoint Merge, and Staleness

- **Unrendered Fields**:
  - Identify the live dashboard component and wire in `rpp_debug`, `spray_controller_state`, `spray_fault_reason`, `alignment_active`, `start_stage`, and `resume_stage`.
- **Per-Waypoint Status (`src/context/TelemetryContext.tsx`)**:
  - Merge the output of `usePointMissionEvents` into `TelemetryContext` (or compose both hooks in one provider) so dashboard consumers don't have to wire it up separately.
- **Staleness Indicator (`src/hooks/useRoverTelemetry.ts`)**:
  - Add a received-at timestamp (`telemetry.ageMs`) and derive a stale-after-Nms flag, distinct from full Socket.IO disconnection.

## 4. Upload → Path → Preview (Push-based Readiness)

The frontend currently discovers path-readiness by polling `/loaded-path`. This should be transitioned to socket pushes.

- **`src/context/SocketEventCoordinator.tsx`**:
  - In `handleMissionModeUpdate`, extract and expose `state` and `state_lower` from the event payload.
- **`src/hooks/useMissionStaging.ts` / `src/context/MissionStagingContext.tsx`**:
  - Consume the new `READY` transition signal to trigger `getLoadedMissionPath()` immediately, avoiding the need for polling.
