# Message for requesting actions from the SAR Opal tablet app
Header header

int8 command # which command will be sent

# any optional parameters of command, such as:
#   string name of sprite to load
string params 

# Commands
int8 INIT_ROUND = 0 # tells game to load animal
int8 START_ROUND = 1 # tells game to do countdown and make buttons active
int8 ROBOT_RING_IN = 2 # tells game that the agent has buzzed in
int8 RESET_NEXT_ROUND = 3 # tells game to clear sprites and get ready for next round 
int8 SHOW_RESULTS = 4 # tells game to show pronunciation results
int8 END_ROUND = 5 # tells game to clean up pronunciation results and sprite from the round
int8 SHOW_GAME_END = 6 # tells game to show to end screen