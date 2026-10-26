uint8 MODE_DISABLED=0
uint8 MODE_ENABLED=1
# Legacy/reserved for source/wire compatibility. No functional LIN controller is started.
uint8 MODE_LIN=2
uint8 MODE_FAULT=3

uint8 mode
bool connected
bool enabled
uint32 fault
