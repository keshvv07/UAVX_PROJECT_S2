# UAV-X Stage 2 software architecture

What actually runs, and where the flowchart in `UAV-X_SITL_Architecture_Flowchart_1.pdf`
needs correcting. Read the second section before using that diagram in the Stage 2 report:
two of its labels describe a link the stack does not use.

## The stack as built

```
                      GAZEBO HARMONIC - one shared world, lock-step physics
                      uavx_forest (organisers' map) or uavx_disaster (quake_town)
                      ground truth - pose, velocity, IMU, GPS, baro, mag, camera, rangefinder
                            |                                        |
         ArduPilot JSON FDM |  UDP 9002 + 10i, one link per UAV      | ros_gz_bridge
         (NOT MAVLink)      v                                        v  camera, rangefinder,
                      ARDUPILOT SITL x N                            navsat, baro, mag, odometry
                      one arducopter process per UAV
                      EKF, control loops, flight modes, geofence
                            |  XRCE-DDS, one shared micro-ROS agent
                            v  /ap/v<SYSID>/... topics and services
 ===========================================================================================
                      ROS 2 HUMBLE
 -------------------------------------------------------------------------------------------
  PER UAV                                      SINGLETONS
   companion            onboard autonomy        radio_net        the emulated mesh radio:
     state machine, ETX routing table,            channel model on ground truth, per-frame
     sector sweep, relay hold, partial             delivery, injected impairments, packets.csv
     ferry, battery and endurance rules          gcs              tasking, coverage, relay
   poi_detector         camera -> ground                           chains, ferry decision,
     HSV blob, pinhole projection with the                         watchdog, re-allocation
     rangefinder height, geolocated report      scenario_manager  mission clock, PoI field,
                                                                   console-driven PoIs
                                                control_panel     the evaluator's controls
                                                mission_monitor   referee: logs, metrics,
                                                                   verdict, figures
 -------------------------------------------------------------------------------------------
  SHARED LIBRARIES (no ROS dependency, unit-tested without a simulator)
   world_map   the map: obstacles, canopy, mission area, geofence, pads, A*, radio shadowing
   coverage    camera swath, sector split, lane paths, resumable sweep progress
   routing     ETX distance-vector to the GCS, from neighbour beacons alone
   relay_plan  anchor spacing from the channel model, LIVE / FERRY / BLIND, handoff geometry
   radio_model the channel equations and the bit error rate
   radio_backend  the seam an external simulator (ns-3) plugs into
   radio_endpoint ARQ, multi-hop forwarding, reverse-path ACKs, store-and-forward
 ===========================================================================================
                      /uavx/events  ->  events.jsonl, timeseries.csv, packets.csv,
                                        summary.json, images/, map3d.png, map2d.png
```

The autonomy only ever reads what a real aircraft would: its own autopilot over DDS, its
own camera and rangefinder, and whatever arrives over the emulated radio. Ground truth goes
to two places that are not part of the swarm - the radio channel model, which needs real
positions to decide what a link does, and the referee, which needs them to judge the run.

## Corrections to the flowchart

| In the flowchart | What the stack does |
|---|---|
| "Gazebo ⇄ ArduPilot (**MAVLink**, per UAV)", and the two arrows labelled *MAVLink: UAV state → its ArduPilot* and *actuator / motor cmds → Gazebo* | That link is ArduPilot's **JSON SITL FDM protocol over UDP** (port 9002 + 10·instance), served by the `ardupilot_gazebo` plugin. MAVLink exists in the run, but only as MAVProxy's own channel on 14550 + 10·instance, used for the bench motor test and for the force-disarm that simulates a motor failure. Relabel these arrows, or the report claims a protocol the simulation does not use. |
| "ROS 2 … **hosts the entire swarm logic**", drawn as one Mission Command Node plus one Status Monitor Node | The swarm logic is deliberately **split**. The GCS node allocates work, plans relay chains and supervises; each UAV's companion runs its own routing, coverage, ferry and energy decisions. It has to be split: most of the search area is beyond the 100 m range, so for much of the mission the GCS *cannot* command the UAV that has to decide. A centralised diagram hides the property the challenge is actually testing. |
| ns-3 as a conditional branch feeding link conditions | The channel is the internal model (`radio_model.py`), which shares its equations and numbers with the Stage 1 simulator and is what the results are validated against. `radio_backend.py` is the documented seam where ns-3 goes; selecting it fails loudly rather than falling back, so a run can never report numbers from a channel it did not use. Q26 permits ns-3, it does not require it. |
| Not shown at all | **radio_net** - the node that actually decides whether a frame arrives. The diagram shows DDS carrying "mission & mode commands" between the GCS and the UAVs, but no swarm traffic crosses DDS: it crosses the emulated radio, which is the whole point. |
| Not shown at all | **scenario_manager** (mission clock, the PoI field; no disturbances of its own), **control_panel** (now required by Q32-Q35), **mission_monitor** (the referee), and the **map** as one artefact shared by Gazebo, the planner, the radio and the referee. |
| "COMMAND STATUS == COMPLETED?" / "LOOP 2 - not completed → hold & re-poll" | Right in spirit, wrong in shape. There is no poll-until-complete loop. The GCS re-plans on a 1 Hz supervise tick and on every event that changes the fleet (a PoI appears, a report arrives, a UAV goes quiet, a battery gets low), and a companion that hears nothing keeps working to its last orders. |
| "MISSION COMPLETE → GCS DASHBOARD REPORT" | The run ends when every UAV is down or the watchdog fires, and the referee writes `summary.json` with a PASS/FAIL per criterion plus the figures. The live dashboard is the control panel, which is for the evaluator during the run, not a report after it. |

Everything else in the flowchart - one ArduPilot SITL instance per UAV, one shared Gazebo
world, DDS as the autopilot interface, the GCS and manual override - matches.

## Communication strategy, in numbers

This is the part worth defending in the report, because two of the organisers' constraints
decide it between them: a 100 m radio range (Q13) and a search area whose far corner is
664 m from the ground station (Q6 plus the shared map's layout).

```
GCS mast -> near tree line       112 m     1 relay for a live link
GCS mast -> area centre          360 m     4 relays
GCS mast -> far corner           664 m     7 relays
```

A hop is `range_for_pdr(0.85) * 0.9` = 90 m, with a shorter first hop (80 m) because it
also climbs from the 10 m ground antenna to the 55 m relay height. So:

- a **live chain** to the near half of the area is affordable with the fleet;
- a live chain to the far corner is not, at any fleet size that Q44 would accept;
- therefore the policy is a **partial ferry**: the relays go as far out as the fleet can
  spare, the surveyor works beyond them and buffers, and when a report is older than
  `ferry_after_s` or its sweep leg ends it flies to the **handoff point** - the nearest
  point to it inside the chain tip's range - and the report multi-hops home from there.

A ferry trip from the far quadrant is about 250 m each way, 50 s at 10 m/s, against a
1200 s battery. It costs the surveyor a round trip and not its place in the sweep, because
the lane index is kept across the detour (`coverage.SweepPlan`).

Role assignment follows from the same arithmetic. Quietly sweeping, one UAV holds the first
anchor and four sweep a quadrant each. When a PoI of priority >= `escort_priority` appears,
the GCS gives it to the **nearest** UAV and turns the rest of the fleet into that UAV's
chain, so an emergency is reported live rather than ferried; the chain is released when the
focus moves on.

## What each node decides, and with what information

| Node | Sees | Decides |
|---|---|---|
| `companion` | its own autopilot, its camera, radio frames that reach it, the last orders and chain it was told about | when to climb, where to fly, which lane is next, when to break off and ferry, when the battery says go home, whether to hold a relay anchor, which neighbour to route through |
| `gcs` | the radio frames that reach it (directly or forwarded), the map | who sweeps which sector, who flies which anchor, LIVE / FERRY / BLIND, who takes an emergency PoI, whether a silent UAV is lost or merely out of range, when to recall |
| `radio_net` | ground truth, the map, injected impairments | whether each frame arrives, when, and why not |
| `scenario_manager` | the scenario, the run seed, UAV status | when each PoI appears, and executes what the control console injects |
| `control_panel` | everything on the ROS graph | nothing - it can only inject disturbances, which is the only power an evaluator grading a run should have |
| `mission_monitor` | ground truth and every event | the verdict |

Under `policy: connected` (Level 5) the split changes in one place each. The `gcs` hands
its tasking to `mission_plan.ConnectedPlanner`, which decides who the one surveyor is, holds
it in `STAGE` until its relay chain reports `on_station`, sends a ferry sortie only to a PoI
the chain cannot reach, and keeps each PoI with one UAV. The `companion` adds a leash: off a
sortie it holds when its route to the GCS goes, then pulls back towards the network. It also
flies the sortie out, survey, back to the handoff point, hand over. The `mission_monitor`
measures the result from ground truth: multi-hop connectivity, concurrent ferries, and PoI
ownership. See `ros2/README.md`, "Level 5: the connected-chain policy".

## The map is one artefact

`uavx_gazebo/maps/<name>.yaml` is read by five consumers, which is what stops the Gazebo
world, the path planner, the radio model and the referee from drifting apart:

- **Gazebo**: the obstacles and the forest canopy become collision and visual geometry, so
  the camera and the rangefinder see what the planner avoids.
- **companion**: A* on a grid at the UAV's flight level, obstacles inflated by the clearance.
  The 726 trees are a canopy *layer*, not 726 obstacles: at 35 m they are nothing to fly
  round, so treating them as obstacles would only make every planner call slow and timid.
- **radio_net**: obstacle attenuation plus foliage loss per metre of canopy a path crosses.
- **gcs**: the same prediction, to place anchors and to judge whether a silent UAV should
  have been audible.
- **mission_monitor**: collisions, clearance, the geofence, and the figures.

The map also carries the geographic origin, and `load_scenario` pins the ENU frame to it.
That is not a detail: the two maps in this repository put their origin in different places
(`quake_town` at the GCS, `forest_rescue` at the centre of the search area, 365 m away), and
a frame that disagrees with the Gazebo world converts every position goal into a latitude
and longitude somewhere else entirely. `test_scenarios.py` checks the map, the world file
and the loaded frame against each other for every scenario.
