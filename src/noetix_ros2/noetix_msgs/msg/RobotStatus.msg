uint8 MODE_DISABLED=0
uint8 MODE_ENABLED=1
# Legacy/reserved for source/wire compatibility. No functional LIN controller is started.
uint8 MODE_LIN=2
uint8 MODE_FAULT=3
# 阻尼阶段仍有驱动输出，区别于完全失能。
uint8 MODE_DAMPING=4

uint8 mode
bool connected
bool enabled
uint32 fault
