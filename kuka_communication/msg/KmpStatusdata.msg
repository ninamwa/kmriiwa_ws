########################################
# CONSTANTS
########################################
# Bumper states (states are combined, when multiple bumpers are pressed)
uint8 BUMPER_FORWARD  = 1
uint8 BUMPER_BACKWARD = 2


########################################
# Messages
########################################
std_msgs/Header  header
string operation_mode
bool ready_to_move
bool warning_field_clear
bool protection_field_clear
