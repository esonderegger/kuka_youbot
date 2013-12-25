# This message communicates the state of the PR2's power board.
int8 STATE_NOPOWER=0
int8 STATE_STANDBY=1
int8 STATE_PUMPING=2
int8 STATE_ON=3

# Preferred over STATE_ON, keeping STATE_ON for backcompat
int8 STATE_ENABLED=3

int8 STATE_DISABLED=4

int8 MASTER_NOPOWER=0
int8 MASTER_STANDBY=1
int8 MASTER_ON=2
int8 MASTER_OFF=3
int8 MASTER_SHUTDOWN=4

Header header
# Name with serial number
string name
# Serial number for this board's message
uint32 serial_num
# Input voltage to power board
float64 input_voltage

# Master States:
# MASTER_NOPOWER, MASTER_STANDBY, MASTER_ON, MASTER_OFF, MASTER_SHUTDOWN

# The master state machine's state in the powerboard
int8 master_state

# Circuit States:
# STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED

# One of the above states
int8[3] circuit_state
# Output voltage of each circuit
float64[3] circuit_voltage

# True if robot should be enabled
#Note - if the wireless run-stop is hit, this will be unobservable
bool run_stop
bool wireless_stop

