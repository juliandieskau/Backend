Header header
float64[] P
float64[3] euler_angles # RPY in deg, ENU
float64[3] pos # UTM in m, ENU
float64[3] vel # m/s, ENU
float64[3] bias_acc # m/s2
float64[3] bias_omega # rad/s
float64[]  scale_factor_vel_odo
int32      driving_mode # 0: forwards, 1: backwards, 2: stopped, 3: unknown
bool       gps_active
bool       fog_active
bool[]     odo_vel_active
bool[]     delta_3d_active
bool[]     delta_2d_active
bool       twist_active
bool       loc_in_map_active
bool       heading_estimator_active
bool       obtained_init_data
bool       obtained_gps
bool       obtained_loc_in_map
