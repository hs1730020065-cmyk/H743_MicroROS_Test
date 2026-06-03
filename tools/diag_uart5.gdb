set pagination off
file Debug/H743_MicroROS_Test.elf
target remote localhost:3333
monitor halt
print/x $pc
print huart5.Init.BaudRate
print huart5.Init.WordLength
print huart5.Init.StopBits
print huart5.Init.Parity
print huart5.gState
print huart5.RxState
print/x huart5.ErrorCode
print sbus_rx_frame_count
print g_sbus_frame_ok
detach
quit
