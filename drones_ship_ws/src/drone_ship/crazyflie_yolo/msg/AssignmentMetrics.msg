std_msgs/Header header

# Labels
string[] drone_ns
int32[] target_ids
int32[] target_class_ids

uint32 num_drones
uint32 num_targets

# World state at time of computing the matrix
geometry_msgs/Point[] drone_pos_world          # size M
geometry_msgs/Point[] target_pos_world         # size N
float64[] target_logdet_p                       # size N  (log(det(P_i)))

# Matrices (row-major), index = j*num_targets + i
float64[] dopt                                # tilde_deltaJ_(j,i)
float64[] dist                                # ||p_i - r_j||
float64[] cost                                # C(j,i) total cost you used in min-cost flow

# Per-drone info (size M)
float64[] sep_penalty                          # φ_j
int32[] primary_track_index                    # index in [0..N-1] or -1
int32[] primary_target_id                      # target_ids[primary_track_index] or -1

# The chosen q* for the primary assignment (world)
geometry_msgs/Pose[] qstar_world               # size M (identity if no assignment)
bool[] has_qstar                               # size M

# Parameters used (so plots are self-contained/reproducible)
float64 eta
float64 beta
float64 gamma
float64 rho
float64 kappa
float64 d_max
float64 r_safe

uint32 hover_l
float64 hover_radius
float64 hover_height
float64 travel_speed
float64 hover_tmax
float64 meas_sigma0
float64 meas_k_range
