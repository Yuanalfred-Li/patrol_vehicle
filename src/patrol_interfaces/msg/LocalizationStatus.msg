std_msgs/Header header

bool valid
int32 gps_status
uint8 nsv1
uint8 nsv2

float64 latitude
float64 longitude
float64 altitude

float64 east
float64 north
float64 yaw_deg

string reason

# 结构化质量状态。控制逻辑不得解析 reason 字符串。
bool gps_data_valid
bool imu_data_valid
bool origin_valid
bool gnss_quality_ok
bool gnss_degraded_candidate

float32 gps_age_sec
float32 imu_age_sec

# 仅用于显示和记录，当前不参与有效性判定。
int32 nav_st
