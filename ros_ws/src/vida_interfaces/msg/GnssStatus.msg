builtin_interfaces/Time timestamp

uint8 RTK_NONE=0
uint8 RTK_FLOAT=1
uint8 RTK_FIXED=2

uint8 fix_type
uint8 rtk_status
uint8 satellites

bool gnss_fix_ok
bool differential_solution
bool corrections_received

float32 horizontal_accuracy_m
float32 vertical_accuracy_m
float32 correction_age_s
