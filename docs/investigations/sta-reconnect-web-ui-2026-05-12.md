# Investigation: Web UI after STA reconnect

## Summary
Investigation in progress. Initial log review shows that after the last STA reconnect on 2026-05-12 at 17:16:30–17:17:01, RTSP traffic resumed and health logs remained healthy, while the user reports the web UI is the remaining broken path.

## Symptoms
- Since the last STA reconnect, streaming/RTSP appears to work.
- Web UI does not work after STA reconnect.
- Log file under review: `docs/All_2026-5-12-17_19_35.csv`.

## Background / Prior Research
No external research required yet; the issue appears confined to workspace logs and firmware HTTP/network lifecycle.

## Investigator Findings
<!-- Pair investigator should append structured analysis here with file:line refs, evidence, and conclusions. -->

## Investigation Log

### Phase 1 - Initial log triage
**Hypothesis:** STA reconnect restores network/RTSP but leaves port-80 WebServer or WebUI route handling unavailable.
**Findings:** The CSV is reverse-chronological. Last reconnect sequence: duplicate `STA reconnect scheduled` at 17:16:30; reconnect attempt at 17:16:31; STA connected with IP `192.168.1.53` at 17:17:01; setup AP scheduled to stop after 120000 ms; RTSP client connected and played at 17:18:52; setup AP stopped at 17:19:01. Health after reconnect reports `net=sta`, valid RSSI, and RTSP stream.
**Evidence:** `docs/All_2026-5-12-17_19_35.csv` lines 3-26.
**Conclusion:** Confirmed the failure is likely specific to the control-plane HTTP/Web UI path, not total Wi-Fi or RTSP failure.

## Root Cause
Pending.

## Recommendations
Pending.

## Preventive Measures
Pending.
