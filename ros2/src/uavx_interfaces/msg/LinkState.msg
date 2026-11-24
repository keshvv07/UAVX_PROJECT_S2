# State of one radio link (undirected) as measured by radio_net.
string a
string b
float32 distance_m
float32 snr_db
float32 pdr             # smoothed packet delivery ratio 0..1 (includes injected loss and BER)
float32 channel_pdr     # the channel alone, before injected loss / BER - what a planner predicts
float32 latency_ms
float32 ber             # bit error rate in force on this link (0 = clean)
float32 extra_loss_db   # obstacle, canopy and jamming attenuation on this path
bool up                 # link usable (hysteresis: up at 0.55 PDR, down below 0.45)
string impairment       # "", "outage", "packet_loss", "ber", "jamming", "obstructed", ...
