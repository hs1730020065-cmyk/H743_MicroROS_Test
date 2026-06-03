set pagination off
file Debug/H743_MicroROS_Test.elf
target remote localhost:3333
monitor halt
set $msp = *(unsigned int*)0x08000000
set $pc = *(unsigned int*)0x08000004
detach
quit
