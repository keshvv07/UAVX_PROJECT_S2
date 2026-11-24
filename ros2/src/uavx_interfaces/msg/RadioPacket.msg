# One frame on the emulated UAV mesh radio.
# Nodes publish frames on /radio/tx; radio_net delivers each one (or drops it) on
# /<dst>/radio/rx according to the link model, injected disturbances and latency.
#
# Single hop vs multi-hop. `dst` is always the LINK-LEVEL next hop: radio_net only ever
# models one link. `final_dst` is where the frame is really going; when it differs from
# `dst`, the node that receives it forwards it on towards `final_dst` using its own
# routing table, decrementing `ttl`. A frame is dropped when ttl reaches 0, and a
# receiver drops one it has already seen (same src and msg_id), so a transient routing
# loop costs frames, never the run.

string src              # original sender node id: "gcs", "uav1", "uav2", ...
string dst              # link-level next hop, or "*" to broadcast to every neighbour
string final_dst        # end-to-end destination; empty means the same as dst
uint32 seq              # per-sender sequence number (PDR accounting, neighbour ETX)
string kind             # HEARTBEAT, STATUS, TASKS, POI_REPORT, TASK_DONE, TASK_RELEASE, RECALL, ACK
string msg_id           # set on reliable frames: the destination ACKs it and duplicates are dropped
string payload          # JSON body
builtin_interfaces/Time stamp   # sim time the ORIGINAL sender created the frame
uint8 attempt           # 1 = first transmission, 2+ = ARQ retransmission
uint8 hops              # hops travelled so far
uint8 ttl               # hops still allowed; 0 = drop instead of forwarding
string[] route          # nodes the frame has passed through, sender first
