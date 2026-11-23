# Spec: softwarespezifikation.md section 5.1 (topic /hw/safety).
# Mirrors the safety state machine that runs in the Nano (D-04).
uint8 BOOT_LOCKED=0
uint8 ESTOP=1
uint8 WAIT_ACK=2
uint8 ARMED=3

std_msgs/Header header
uint8 state
bool estop_pressed
bool relay_on
bool block_forward
bool block_backward
bool pi_link_lost
bool software_stop
uint16 flags
