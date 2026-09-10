# The A/B split is a receiver **configuration** split — 2026-09-09

Scope: receiver-side only. Everything here is decoded from `gps_dump` (the raw
Septentrio serial traffic PX4 captures) and from ULog parameters. No EKF2, no
controller, no backend.

Dataset: `Estimator_Compare/{A,B}` plus every other 2026-09-08 ULog that carries a
`gps_dump`. Tool: `scripts/gps_dump_sbf_report.py` (now prints the fingerprint
described below). Wall times are reconstructed from embedded GNSS UTC, never from
ULog filenames (CLAUDE.md hard rule).

## The finding

**Every 2026-09-08 log in which the FCU serial link carried *only* the seven SBF
blocks PX4 asks for was positionally good. Every log in which the receiver also
pushed blocks of its own onto that link was wrong by 150–210 mm.** 4 bad / 5+ good,
no exceptions in the logs where physical truth is known.

| IST | logs | receiver uptime at start | SBF blocks on FCU link | position |
|---|---|---|---|---|
| 11:03–11:06 | `log_12` | — | *no `gps_dump`* | **GOOD** — 10.1 mm raw from surveyed P0001 |
| 12:33–16:28 | `log_15`, `log_17`, `log_18` | **16–18 s** (fresh boot each) | **10** = PX4's 7 **+ BaseVectorGeod + PosCovGeodetic + AuxAntPositions** | **BAD** — `log_15` +192 mm step, `log_18` −210 mm |
| 17:05–17:31 | `log_20` … `log_31` (group A) | 2245 → 3780 s (one boot, no restart) | **7 — exactly PX4's `sso` set** | **GOOD** — 9–42 mm |
| 19:14–19:25 | `log_47`, `log_48` (group B) | 2678 → 3340 s | **14** = PX4's 7 **+ BaseVectorGeod, PosLocal, PVTSupport, PVTSupportA, PosCovGeodetic, AuxAntPositions, EndOfAtt** | **BAD** — stable +120…+158 mm North |

### Why the block set is a valid fingerprint of receiver configuration

PX4 1.16.2 (`src/drivers/gnss/septentrio/septentrio.cpp`, pinned commit
`54f0455ffcd755534539a7cf33a09a20bf71d29d`) has a **fixed** block list:

```
sso,Stream<N>,COM<n>,none,off                       # clears ONLY its own stream
sso,Stream<N>,COM<n>,PVTGeodetic+VelCovGeodetic+DOP+AttEuler+AttCovEuler
                     +EndOfPVT+ReceiverStatus,<rate>
```

PX4 can never *add* a block outside those seven, and it clears only
`SEP_STREAM_MAIN` (= 1 here). So any additional block arriving on the FCU port
comes from a **second stream that the receiver owns in its own configuration**.
The fingerprint therefore answers one question directly: *was the receiver running
extra output configuration of its own on the autopilot link?*

## What PX4 stops commanding at `SEP_AUTO_CONFIG = 0`

Group A ran `SEP_AUTO_CONFIG = 1` (11/11), group B ran `0` (2/2). The only other
parameter that differs anywhere across the A/B boundary is `EKF2_GPS_YAW_OFF`
(0 → 180), which is EKF2-side and cannot move a raw GNSS fix.

With `SEP_AUTO_CONFIG = 0`, PX4 sends **nothing at all** past baud/port detection
(`configure()` returns early). These stop being commanded — each then falls back
to receiver NVRAM:

| command | what PX4 sends when enabled | note |
|---|---|---|
| `sso,Stream1,COMn,none,off` | clear its stream | |
| `sdio,COMn,Auto,SBF` | input type **Auto**, output SBF | governs how RTCMv3 on this port is accepted |
| `sto,<yaw>,<pitch>` | `sto,0.000,0.000` (`SEP_YAW_OFFS`/`SEP_PITCH_OFFS` = 0) | |
| `srd,high,UAV` | **hard-coded `UAV`**, level `high` | receiver dynamics profile |
| `sso,Stream1,COMn,<7 blocks>,msec100` | the block list above | |
| `sga,MultiAntenna` | attitude source | |
| `ssu,<constellations>` | **not sent** — `SEP_CONST_USAGE = 0` in *both* groups | |

⚠ **`srd` conflicts with the manual-config architecture.** PX4 forces
`srd,high,UAV`. The receiver was later hand-set to `Moderate, Pedestrian`. If
`SEP_AUTO_CONFIG` is ever returned to 1, PX4 silently overwrites that back to
`high, UAV` at every driver start. Dynamics feeds the receiver's motion model and
therefore its ambiguity-fixing confidence — this is the single most plausible
causal candidate among the six, and it is *not* observable in SBF.

## What is ruled out

- **`SEP_AUTO_CONFIG` alone is not the discriminator.** `log_15`, `log_17` and
  `log_18` all ran `SEP_AUTO_CONFIG = 1` and were still 10-block and still wrong.
  It is the block set, not the parameter, that separates good from bad.
- **Link saturation is not the mechanism.** Measured from `gps_dump` timestamps:
  receiver→FCU 2.9 kB/s in the good group vs 6.3 kB/s in group B. Even against
  115200 baud that is 30% → 60%. The wider block set doubles the load and still
  does not saturate the link, so the fingerprint is a **marker of which
  configuration was active**, not a bandwidth cause in itself.
- **BeiDou is not the discriminator** (already retracted in CLAUDE.md
  §2026-09-09, re-confirmed here). Bad logs ran BeiDou at 94.2 / 95.3 / 99.7 %
  (`log_18` / `log_17` / `log_15`); good group-A logs ran it at 19–100 %.
- **Antenna ARP / phase-centre is unchanged throughout.** `PVT Misc` is 0x50/0x60
  in all 14 logs, good and bad, in the same proportion. Base antenna descriptors
  (RTCM 1007/1008/1033) are absent in all of them.
- **RTCM CRC failure rate does not track the split** — 0–8.8 % in the good group,
  1.4–10.3 % in the bad. Consistent with `gps_dump` buffer drops, not a
  discriminator.
- **`ReceiverStatus.RxError` bit 3 (`SOFTWARE`) is set in every log**, good and
  bad, as it has been all along. `ExtError = 2` (DIFFCORR) appears in both groups
  and does not track the split either.

## The 37-minute window that matters

The receiver was power-cycled at roughly **16:29 IST** (group A starts at uptime
2245 s at 17:05:47, and `log_18` runs to 16:28:01). It came back **without** the
extra stream on the FCU port — and the position error went from ±210 mm to
9–42 mm and stayed there for the whole 17:05–17:31 session.

Before that, three consecutive boots (12:33, 14:46, 15:12 — each log starts at
uptime 16–18 s) all came back **with** the extra stream. After it, the 18:29 boot
came back with an even wider set.

**So the receiver does not boot into a deterministic output configuration.** That
is the same pathology found live on 2026-09-09 (`Stream1` bound to `USB1` rather
than `COM3`; `spm` stuck in `Static`): running config and Boot config disagree.

## Open question for the operator — highest value in the dataset

**What was done to the receiver between 16:28 and 17:05 IST on 2026-09-08?**
That window is the only thing separating the worst runs of the day from the best,
and no log can answer it. A power cycle alone does not explain it — the three
earlier boots that day came back bad.

## Next steps, receiver-side only

1. **Dump the receiver's full configuration, both copies.** `lstConfigFile,Current`
   and `lstConfigFile,Boot`, and diff them. A non-empty diff is the whole
   non-determinism story. Save with `eccf,Current,Boot` once correct.
2. **Record the settings SBF cannot show**, since the causal one is among them:
   `gst` / `gsu` (tracking + usage), `srd` (dynamics), `sdio` (per-port I/O),
   `sem` (elevation mask), `smm` (multipath), `spm` (PVT mode + mask), `sao`
   (antenna), and `lif,error` — which also reports and clears the standing
   `RxError` `SOFTWARE` bit.
3. **Decide the `srd` conflict explicitly.** Either keep `SEP_AUTO_CONFIG = 0`
   permanently (current architecture — then PX4 never touches dynamics and the
   manual `Moderate, Pedestrian` stands), or accept `high, UAV`. Do not leave it
   ambiguous; the setting silently flips with that one parameter.
4. **Re-test with the fingerprint as the gate.** Run
   `python3 scripts/gps_dump_sbf_report.py <log.ulg>` and read the
   `config fingerprint:` line before trusting any position from that log.

## Reproducing

```bash
python3 scripts/gps_dump_sbf_report.py \
  --reference 'Estimator_Compare/mission_P1_ref.csv' \
  'Estimator_Compare/A/'*.ulg 'Estimator_Compare/B/'*.ulg
```

The `config fingerprint:` line is new in this commit. It compares the CRC-checked
set of SBF blocks actually emitted against `PX4_SSO_BLOCKS`, the exact list from
the pinned PX4 source. Layouts remain size-checked against the receiver's own
`Length`/`SBLength` — no hand-rolled offsets (CLAUDE.md hard rule).
