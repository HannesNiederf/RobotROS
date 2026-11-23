# Spec: softwarespezifikation.md section 5.1 (topic /hw/hub_state).
uint8 IDLE=0
uint8 MOVING_UP=1
uint8 MOVING_DOWN=2
uint8 TOP=3
uint8 BOTTOM=4
uint8 FAULT=5

std_msgs/Header header
uint8 state
bool limit_top
bool limit_bottom
uint8 fault               # firmware protocol: 0 none, 1 timeout, 2 both limit switches active
