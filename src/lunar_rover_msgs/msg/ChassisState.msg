std_msgs/Header header

uint8 MODE_UNKNOWN=0
uint8 MODE_DISABLED=1
uint8 MODE_MANUAL=2
uint8 MODE_AUTONOMOUS=3
uint8 MODE_FAULT=4

uint8 mode
uint32 fault_bits
bool estop
bool enabled
bool watchdog_ok

