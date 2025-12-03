# Multi-leg Robot with invers kinematic
Robot base to develop a multi legged Robot (3-n) with invers kinematic.
Software is made for EPS32 Dev Boards and an iBus-RC-Reciever like FlySky.

## Build your own
Please decide how many Legs your Robot should have. This is designed for a circular shape so all first motors should be places with the same distance to each other and the same distance to the center of mass.
I think it is possible to build robots from 4 to 8 legs. 

### Stuff to buy
* 3 * Leg-Count ST3020 Servomotors
* Serial Servo Controller-board
* ESP32 Dev Board
* FlySky 6/10 Channel Remote + Reciever

### 3D printing
Inside of the STL Folder you find files for 3D printing.
At the end of the filename you will find a "x5" or similar. 
So with a "x3" you have to print this part 3 times.

### Prepare the Servos
Use my programmer 
https://github.com/JeanetteMueller/SerialServoIdProgrammer
to set all the 1-n ids. 

### Install Libraries
* https://github.com/derdoktor667/FlyskyIBUS
* https://github.com/workloads/scservo
