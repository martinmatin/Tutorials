#Printed Circuit Boards

Three types of printed circuit boards were developed for our two robots. The following section presents these boards, it utilities and it conception 
and gives you all the necessary elements to understand, recreate and use it. 

##Starting Board
Inspired by past version of the starting board

The utility of this PCB is to gather the component serving to the start sequence. Normally this is the only interface with the robot during the 
competition.

First, we used three switches for configuring three mains points of our robot : 

* Team configuration
We have to setup the starting side for the next match to the robot. It behaviour is different if the robot play as the green or the orange team. 
Consequently, this switch set this point before the match start. 

* Strategy Configuration
It makes possible to choose between two kinds of implemented strategies before the match. For example, just before the start, we have the
possibility of aggressive or defensive strategy, function of the opponent. 

* Initialisation Switch

This switch launches an initialisation sequence. After the power up of the robot and before its first move, we wanted to have the possibility to check 
all different robot's systems. This switch indicates with a DEL if all systems are operational. This is a kind of automatic check list before the match 
and the first move of the robot. 

All these switches are linked to two DEL. First one to electrically confirm switches' position and the associated setting and the second one used to 
have the response of the robot operating system. With these two DELs, we are sure that our robot understood all these parameters.

### Useful information
* Correspondence

Strategy  

Pin 1 - UP   = GREEN = 0 (out raspberry) - DOWN  = RED = 1 (out raspberry)

Initialisation

Pin 2 - UP   = GREEN = 0 (out raspberry) - DOWN  = RED = 1 (out raspberry)

Team

Pin 3 - LEFT = GREEN = 1 (out raspberry) - RIGHT = RED = 1 (out raspberry)

Start 

Pin 4 - piece IN     = 0 (out raspberry) - piece OUT   = 1 (out raspberry)

* Switch placement

The emplacement is taken when the splayed side is on the right then : 

1. Strategy switch is in the upper left corner 
2. Initialisation switch is on the right side of the strategy switch 
3. Team switch is in the lower left corner 

Finally, the starter switch is plugged on the board. It consists of a relay, sending a binary information to the ROS operating system. Linked to a 0.5m
cable, this system allows to launch the match sequence pulling on it. 

## Motor Board

The goal of this board is to bring individual intelligence for every motor used on our robots. It allows to associate a microcontroller (ATmega328)
with each motor. This main component is used for two aspects : regulation and communication. 

### Regulation

For the smaller robot, the operating system send speed consign to the four motors. Consequently, the goal for the motor is reaching this speed in a 
minimum of time and after conserving this value. The regulator implemented in the microcontrollers is Proportional Integral Derivative regulators. 
It's parameters allow to influence the way to consign reaching for the motor. Thanks to it, each motor, individually can follow the ROS order with a 
maximum of precision. 

PID's parameters were found with experimentation and all sources used for implementation are in the code file. 

### Communication

As said in the previous part, this is the operating system ROS the main brain of the robot. Consequently, this brain has to communicate with its slave.
The communication protocol used on our robots is SPI. It induces that a bus has to be built between all motor and ROS. Effectively, a broker gathers 
the signal for the motor but in any case each motor has to be linked to the bus. A role of the board is this connection via a SPI connector.

The commented code can be found in the root folder. For more information about the SPI communication, go to the SPI section.

## Arduino Communication Shield - SPI Broker
Motor command communication consists of a two-level interface. ROS communicate via ROS Serial (explained in the appropriate section) implemented on an 
arduino UNO and this arduino handle the communication with the four individual motors. To help for connections, we designed an arduino shield gathering
simply all SPI bus connections. 

Again, you will find more explanation about the SPI communication in the right section and the basis of the ROS Serial use in the related section as 
already said. 

## Annexe

All the boards adressed in this section are linked with the full Altium project in the root folder. It is the same for code and STL file for necessary 3D pieces.

## If we had to redo it 

If we had to design another robot with technologies described, there are some decision that we would change. 

* PCB Design

The ECAM installation for PCB printing is very useful for prototyping. It is not comparable with PCB that specialised company can create. So, our advice
is the following, as soon as a board is functional and definitive, send it to have a proper and reliable board. Moreover, try to reach this step as soon
as possible. 

* Connection 

We have underestimated the connection on the different board. Regarding the JST connector, they are solid but hard to disconnect and cable creation is long withtout the right tool. About the 10 pins shrouded header, they are not reliable enough. Opposite to the JST, disconnection is too easy. We really advise against
its use. They were the source of a lot of problems.

