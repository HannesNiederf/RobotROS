# Spec: softwarespezifikation.md section 5.1 (topic /hw/power).
std_msgs/Header header
float32 v_batt
float32 v_charger
float32 v_rail
float32 i_charge          # A, positive = charging
bool charger_present
bool charge_relay_on
float32 soc_estimate      # -1 = unknown
