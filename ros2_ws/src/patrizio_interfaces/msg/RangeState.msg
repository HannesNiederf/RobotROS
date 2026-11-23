# Spec: softwarespezifikation.md section 5.1 (topic /hw/range).
# front_mm / rear_mm: distance in mm, 65535 = no echo.
uint8 CLASS_UNKNOWN=0
uint8 CLASS_FLOOR=1
uint8 CLASS_CLIFF=2
uint8 CLASS_OBSTACLE=3

std_msgs/Header header
uint16 front_mm
uint16 rear_mm
uint8 front_class
uint8 rear_class
