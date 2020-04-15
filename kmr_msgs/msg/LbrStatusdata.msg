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
bool ready_to_move
bool is_lbr_moving
bool lbr_has_active_command
bool lbr_safetystop