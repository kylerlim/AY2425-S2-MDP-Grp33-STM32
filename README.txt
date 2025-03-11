A repo for STM32 portion of MDP

1. Connect to microUSB port behind OLED screen

2. STM32 receives commands of 5 words from RPi/UART connection

List of commands:

[Command] [Reserved/ Don't Care] [H] [T] [O]

List of acceptable [Command]s: 

[S]: Move Forward (cm)
[B]: Move Backward (cm)
[L]: Move Left Forward (angle in degrees)
[R]: Move Right Forward (angle in degrees)
[V]: Move Left Backward (angle in degrees)
[W]: Move Right Backward (angle in degrees)

[H] [T] [O] are the hundreds, tens, and ones place of the parameter

e.g. [S] [X] [1] [0] [0] -> move forward 100cm
e.g. [R] [X] [0] [9] [0] -> turn right 90 degrees