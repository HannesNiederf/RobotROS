# Spec: softwarespezifikation.md section 5.1 and 9.3.
string TYPE_TRANSPORT="TRANSPORT"
string TYPE_GOTO_POINT="GOTO_POINT"
string TYPE_GOTO_DOCK="GOTO_DOCK"

string STATE_QUEUED="QUEUED"
string STATE_ACTIVE="ACTIVE"
string STATE_DEFERRED="DEFERRED"
string STATE_COMPLETED="COMPLETED"
string STATE_CANCELLED="CANCELLED"

string STEP_PICKUP_NAV="PICKUP_NAV"
string STEP_ENGAGE="ENGAGE"
string STEP_DEST_NAV="DEST_NAV"
string STEP_RELEASE="RELEASE"
string STEP_DONE="DONE"
string STEP_NAV="NAV"

string ORIGIN_USER="USER"
string ORIGIN_SYSTEM="SYSTEM"

uint32 id
string type
uint8 priority            # 0-9, 9 = highest, default 5 (D-14)
uint32 source_station_id
uint32 dest_station_id
geometry_msgs/Pose2D goal
string state
string step
string error_code
string error_detail
uint8 attempts
string created_at
string origin
