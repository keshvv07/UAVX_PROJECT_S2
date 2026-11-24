# Grand Challenge 1, Stage 2: every answer, and where it is handled

The 48 answers from the technical clarification session with Prof. Sujit, each mapped to the
code, configuration or test that satisfies it - and, where something is not satisfied yet,
said so plainly rather than left to be discovered during evaluation.

Status: **done** = implemented and checked; **open** = not yet; **n/a** = nothing for us to do.

## 1. Simulation environment and submission

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q1 | Any simulator is allowed if the physics are right, but the environment must be packaged in Docker | **open** | Gazebo Harmonic + ArduPilot SITL gives the physics (real firmware, lock-step). **No Dockerfile yet** - the single biggest outstanding item; see "Outstanding" below |
| Q2 | Isaac Sim or another simulator is acceptable | n/a | Gazebo Harmonic, with the organisers' own world geometry |
| Q3 | Team PUSHPAK runs it on their GPU hardware | **open** | depends on Q1; `headless_rendering:=true` already exists for a container without a display |
| Q4 | Submit source, execution scripts, physics models, communication models, config and logs | partly | all present in the repository (`ros2/`, `run_level.sh`, `setup_ws.sh`, `radio.yaml`, run outputs); packaging is Q1 |
| Q5 | They evaluate the logs with their own scripts | **done** | `events.jsonl`, `timeseries.csv`, `packets.csv`, `summary.json`, `target_field.json` - all machine-readable, documented in `ros2/README.md` |

## 2. Mission area, fleet and flight constraints

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q6 | 500 m x 500 m, not 1 km x 1 km | **done** | `maps/forest_rescue.yaml` `mission_area:`; `test_forest_map.py::test_search_area_is_500_by_500_on_the_origin` |
| Q7 | 20 minute maximum endurance | **done** | `companion.FakeBattery(endurance_min)`, `gc1.yaml` `battery.endurance_min: 20`; referee criterion `endurance_respected` |
| Q8 | Battery parameters may vary; the algorithm must adapt | **done** | endurance is one number that every decision derives from (return-home, relay eligibility, whether a ferry trip is affordable). the control console's endurance control changes it mid-run, and the panel has a control for it |
| Q9 | No minimum return reserve prescribed | **done** | `reserve_pct` is our own safety margin, not a limit we claim |
| Q10 | Fleet size is the team's decision; a large fleet is discouraged | **done** | five, justified by the relay arithmetic in `docs/architecture.md`; `test_scenarios.py::test_gc1_flies_the_organisers_map_with_four_to_five_uavs` pins 4-5 |
| Q11 | UAV type is the team's choice, to be justified | **done** | `uavx_quad`: an ArduPilot Iris with a nadir camera and a rangefinder - the sensors Q14 makes mandatory and nothing it cannot lift |
| Q12 | 10 m/s recommended, not fixed | **done** | `gc1.yaml` `ap_params: {WP_SPD: 10, ...}`, kept per-scenario so Levels 1-4 keep the 5 m/s they are scored against; referee checks truth speed |
| Q13 | 100 m maximum inter-UAV range | **done** | `radio.yaml` `max_range_m: 100`, a hard zero beyond it; referee criterion `comm_range_limit`; `test_scenarios.py::test_radio_is_the_organisers_radio` forbids any scenario widening it |

## 3. Sensors, camera and detection

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q14 | GPS and camera mandatory | **done** | both on `uavx_quad`; `bench_check` measures every sensor's rate before anything flies |
| Q15 | Extra sensors allowed | **done** | a downward rangefinder is fitted and used for the geolocation height |
| Q16 | Configuration changes allowed, to be justified | **done** | `models/uavx_quad/model.sdf`; the launch strips the camera or rangefinder for a UAV that does not need one |
| Q17 | No prescribed field of view | **done** | 1.2 rad horizontal, 640x480; the coverage planner derives the lane spacing from it rather than from a guess (`coverage.swath_m`) |
| Q18 | No prescribed altitude; balance coverage against detection | **done** | 35 m survey height: a 48 m x 36 m footprint, 34 m lanes at 30 % overlap, one quadrant in about 200 s. Relays hold 55 m, each UAV transits at its own height |

## 4. Points of interest

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q19 | PoIs have predefined spawn locations and times | **done** | `targets:` draws the field the way the organisers' generator does (10 sites, 15 m apart, clear of trunks), written to `target_field.json`; a `pois:` list takes their published positions instead |
| Q20 | Not all are available at the start | **done** | `at_start: 3`, the rest through the first 700 s. The swarm searches rather than flying to known coordinates - `sweep` tasks, and a find while sweeping is reported as a sighting the GCS then matches |
| Q21 | No hard detection deadline, but time counts | **done** | `POI_APPEARED` -> `POI_DETECTED` is in the event log; `since_known_s` on completion |
| Q22 | No expiry window | **done** | nothing expires |
| Q23 | 10 s from detection to the GCS | **done** | referee criterion `live_report_latency_s`, applied where a link existed at the moment of detection; everything else must still arrive, by relay or ferry (`ferry_delivered`) |
| Q24 | The image must be part of the timely report, not post-processed | **done** | `poi_detector` writes the snapshot before the report is sent and the report carries its path; the referee records the image against the PoI |

## 5. Communication

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q25 | Reliability judged from our communication model | **done** | `radio_model.py` / `radio_net.py`; every frame logged in `packets.csv` with its route and why it failed |
| Q26 | An external simulator such as ns-3 may be layered in | **done, by decision** | internal model kept as the validated one; `radio_backend.py` is the documented seam, and selecting `ns3` fails loudly rather than pretending |
| Q27 | They will change communication parameters during evaluation | **done** | the control console changes range-independent impairments live; `clear_impairments` puts it back |
| Q28 | Temporary disconnections permitted | **done** | `comm_outage`, scheduled or hidden or by hand |
| Q29 | Latency measured by GPS time synchronisation | **done** | every frame carries the original sender's sim time, which is the shared GPS clock; latency is measured at the destination and survives any number of hops |
| Q30 | PDR and communication performance from the model and logs | **done** | per-link and per-kind counters, end-to-end report latency, connectivity availability, hop counts, forwarded-frame counts |

## 6. Dynamic disturbances and faults

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q31 | Dynamic disturbances during the mission | **done** | No disturbance is scripted or generated by the swarm; the evaluator injects them live from the control console (`control_panel.py`), and `scenario_manager` refuses any other source |
| Q32 | A centralised control panel that can switch off an individual UAV | **done** | `control_panel.py`, per-UAV kill switch; `http://localhost:8089` |
| Q33 | A slider for bit error rate, 0 to 1, responded to dynamically | **done** | BER is a first-class impairment (`frame_success(ber, frame_bits)`), with the slider and a live readout of what fraction of frames survive |
| Q34 | Both config files and a UI | **done** | the web control console is the only way in: it publishes `/scenario/inject_request`, which `scenario_manager` accepts only from the console, so one code path and one audit trail; scenario files hold no disturbances |
| Q35 | Why the panel matters: change things without restarting | **done** | every control takes effect on the next radio update, and is logged as a `DISTURBANCE` the referee scores |

## 7. GPS, navigation, scenarios

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q36 | No GPS noise in this challenge | **done** | none injected; the EKF error against ground truth is still measured in `bench_check` |
| Q37 | Evaluation maps will be like the one shared | **done** | the shared one is used: `tools/gen_forest_site.py` generates the map and world from the organisers' `roi_forest` coordinates and `site.py` layout |
| Q38 | Slower or faster than real time is fine | **done** | ArduPilot and Gazebo run in lock-step, so the verdict does not depend on wall-clock speed; the real-time factor is reported |

## 8. Onboard computing

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q39 | Onboard AI is allowed | **done** | detection and geolocation run per-UAV in `poi_detector`, as they would on a companion computer |
| Q40 | No power limit prescribed | n/a | - |
| Q41 | GPUs available | **done** | rendering uses them when present; `headless_rendering` for a container without a display |

## 9. Logs and evaluation

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q42.1 | UAV positions and trajectories | **done** | `timeseries.csv` at 2 Hz from ground truth; tracks in the figures |
| Q42.2 | Inter-UAV communication range | **done** | `communication.inter_uav_range_m` (max and mean while a link was up, against the 100 m limit) |
| Q42.3 | Battery consumption and endurance | **done** | per UAV: used %, % per minute, flight time, endurance margin |
| Q42.4 | Mission start and completion times | **done** | `metrics.mission` - clock start, first airborne, last task done, recall, end, durations |
| Q42.5 | Communication reliability | **done** | status PDR, frame PDR, latency mean and p95, connectivity availability, downtime per UAV |
| Q42.6 | PoI detection and reporting performance | **done** | per PoI: who found it, when, geolocation error, report latency, hops, image |
| Q43 | Judged on reproducibility of the submitted environment | partly | seeds make a run replayable (`--seed N`); the container is Q1 |
| Q44 | Scalability and efficiency, not brute force | **done** | five UAVs with the arithmetic to justify it; roles decided at run time; `docs/architecture.md` |

## 10. Design strategy

| Q | Requirement | Status | Where |
|---|---|---|---|
| Q45 | Fleet size is part of the design | **done** | see Q10 |
| Q46 | No prescribed algorithm | **done** | greedy earliest-finish allocation, ETX distance-vector routing, geometric relay placement, boustrophedon coverage - all ours, all unit-tested |
| Q47 | No business plan needed | n/a | - |
| Q48 | Survey, detect, coordinate, communicate, handle faults, manage energy, show reliability | **done** | the `pass:` block of `gc1.yaml` asks the referee for exactly this list |

## Outstanding

1. **Docker packaging (Q1, Q3, Q4, Q43).** The one item that would make the submission
   incomplete on its own. It needs: ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL, the
   workspace built, a one-command entrypoint, the control panel's port published, a mounted
   logs volume, and `headless_rendering` on by default. Deferred by choice to the next
   session.
2. **ArUco markers.** The organisers' targets carry ArUco 4x4_50 plates readable to about
   25 m; `poi_detector` finds a red ground marker by colour. The detection pipeline and the
   geolocation are unchanged either way, but reading their plates (and reporting the marker
   id) would match their world exactly. `cv2.aruco` needs `opencv-contrib`.
3. **A full Gazebo run of `gc1`.** Everything here is checked by 114 offline tests and by
   the geometry; the end-to-end behaviour of five UAVs in the forest has not been flown
   yet. Expect to tune `ferry_after_s`, `fail_timeout_s` and `lost_timeout_s` on the first
   few runs.
4. **Licensing note.** The map is generated from the organisers' *coordinates* with our own
   geometry; no file from their LGPL-3.0 `uavsim` repository is copied into this MIT one. If
   their SDF or meshes are ever vendored directly, the licence needs revisiting first.
