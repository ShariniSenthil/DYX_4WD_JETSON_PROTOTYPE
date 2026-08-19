FRONTEND HRMS / VRMS PATCH
==========================

THIS PATCH IS FOR THE CURRENT FRONTEND STRUCTURE
------------------------------------------------
Repo structure checked against:
  ShariniSenthil/DYX_GCS_Frontend

Files changed:
  src/services/rtkService.ts
  src/components/missionreport/RTKInjectionScreen.tsx

BACKEND FIELDS USED
-------------------
GET /api/telemetry/latest

telemetry.gps.px4_hrms_mm
telemetry.gps.px4_vrms_mm
telemetry.gps.px4_estimator_flags
telemetry.gps.px4_estimator_healthy

The existing RTK endpoint remains responsible for:
  GNSS fix
  RTK_FIXED / RTK_FLOAT
  correction freshness
  satellites
  correction age

IMPORTANT BEHAVIOR
------------------
PX4 estimator HEALTHY does not mean RTK FIXED.

Example:
  fix_type = 3 / 3D_FIX
  px4_estimator_healthy = true

Frontend should show:
  GNSS Fix: 3D FIX — NO RTK
  HRMS: live value
  VRMS: live value
  PX4 Estimator: Healthy

The RTK status pill must NOT become RTK Fixed until the existing RTK
status says fix_type=6 / rtk_fixed=true with fresh corrections.

APPLY ON WINDOWS
----------------
1. Extract this ZIP into your frontend repo, for example:

   C:\Users\SHARINI\Downloads\DYX_GCS_V-refactor\patch_hrms_vrms

2. Open CMD in the repo root:

   cd C:\Users\SHARINI\Downloads\DYX_GCS_V-refactor

3. Apply:

   python patch_hrms_vrms\apply_frontend_hrms_vrms.py

4. Verify source:

   python patch_hrms_vrms\verify_frontend_hrms_vrms.py

5. Type-check if your project supports it:

   npx tsc --noEmit

6. Start your normal dev-client workflow:

   "%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe" reverse tcp:8081 tcp:8081

   npx expo start --dev-client --localhost --clear

NO PREBUILD IS REQUIRED
-----------------------
This patch changes only TypeScript/React Native code. It does not add a
native Android dependency, so you do not need to prebuild again just for
HRMS/VRMS.

BACKUPS
-------
The patch automatically copies the original files to:

  %USERPROFILE%\DYX_frontend_backups\hrms_vrms_<timestamp>\

EXPECTED UI
-----------
GNSS Fix             3D FIX — NO RTK
Satellites           28

HRMS (PX4 EKF 1σ)    8.9 mm
VRMS (PX4 EKF 1σ)    8.4 mm
PX4 Estimator        Healthy

Correction           Fresh / Not Fresh
Correction Age       x.xx s

When RTK returns:
GNSS Fix             RTK FIXED

The live HRMS/VRMS numbers will continue changing.

SPRAY / MARKING
---------------
HRMS/VRMS are display/estimator-confidence information only.

They do not change the waypoint marking gate.
