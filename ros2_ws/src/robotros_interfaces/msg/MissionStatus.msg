# Spec: softwarespezifikation.md section 5.1 and 9.2 (topic /mission/status).
string MODE_NORMAL="NORMAL"
string MODE_MAPPING="MAPPING"
string MODE_MANUAL="MANUAL"

string STATE_BOOTING="BOOTING"
string STATE_LOCKED="LOCKED"
string STATE_LOCALIZING="LOCALIZING"
string STATE_IDLE="IDLE"
string STATE_AWAIT_RESUME="AWAIT_RESUME"
string STATE_GOTO_PICKUP="GOTO_PICKUP"
string STATE_ENGAGE_CART="ENGAGE_CART"
string STATE_GOTO_DEST="GOTO_DEST"
string STATE_RELEASE_CART="RELEASE_CART"
string STATE_GOTO_POINT="GOTO_POINT"
string STATE_GOTO_DOCK="GOTO_DOCK"
string STATE_DOCKING="DOCKING"
string STATE_CHARGING="CHARGING"
string STATE_WAITING_MAINS="WAITING_MAINS"
string STATE_UNDOCKING="UNDOCKING"
string STATE_HELD_WITH_CART="HELD_WITH_CART"
string STATE_NEEDS_OPERATOR="NEEDS_OPERATOR"
string STATE_PAUSED_SAFETY="PAUSED_SAFETY"
string STATE_MAPPING="MAPPING"
string STATE_MANUAL="MANUAL"

std_msgs/Header header
string mode
string state
uint32 active_order_id
string step
float32 progress
string message_key
string error_code
bool lifted
float32 loc_sigma_xy
string waiting_reason
uint32 queue_length
