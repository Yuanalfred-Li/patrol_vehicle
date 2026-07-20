uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 FAILED=3

std_msgs/Header header
uint8 state
string task
string message
float32 progress
