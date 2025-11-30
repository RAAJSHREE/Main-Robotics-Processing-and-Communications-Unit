builtin_interfaces/Time stamp
string robot_id
string safety_state

# Battery and power
float32 battery_pct
float32 voltage_v
float32 temperature_c

# Position
float32 position_x
float32 position_y
float32 position_theta

# P2 Interface
float32 p2_canopen
float32 p2_ethernet
float32 p2_lvds
float32 p2_optical
string p2_status
float32 p2_cpu_percent
float32 p2_mem_percent
float32 p2_disk_free

# P3 Interface
float32 p3_canopen
float32 p3_ethernet
float32 p3_lvds
float32 p3_optical
string p3_status
float32 p3_cpu_percent
float32 p3_mem_percent
float32 p3_disk_free

# P4 Interface
float32 p4_canopen
float32 p4_ethernet
float32 p4_lvds
float32 p4_optical
string p4_status
float32 p4_cpu_percent
float32 p4_mem_percent
float32 p4_disk_free

# System state
bool led
bool estop
