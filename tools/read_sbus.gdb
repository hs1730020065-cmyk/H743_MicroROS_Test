set pagination off
file Debug/H743_MicroROS_Test.elf
target remote localhost:3333
monitor halt
print g_sbus_frame_ok
print g_sbus_failsafe
print g_sbus_estop_active
print g_sbus_data.ch[0]
print g_sbus_data.ch[1]
print g_sbus_data.ch[2]
print g_sbus_data.ch[3]
print g_sbus_data.frame_lost
print g_sbus_data.failsafe
print g_sbus_control
detach
quit
