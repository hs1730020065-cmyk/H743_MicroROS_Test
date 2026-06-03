set pagination off
file Debug/H743_MicroROS_Test.elf
target remote localhost:3333
monitor halt
print/x $pc
x/4wx 0x08000000
x/4wx 0x1ff00000
detach
quit
