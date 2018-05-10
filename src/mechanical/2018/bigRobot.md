# The big robot specifications
![alt text](mechanical/2018/bigrobot_render.PNG)
## Fusion 360

The entire robot is designed in Fusion 360 and is available on the Eurobot cloud. The final version is called **SHARK FINAL V17**

Here is the link to download de fusion projet : [BigRobotModel](https://a360.co/2wA0er0)

## The goal of the 2018 edition

![alt text](mechanical/2018/bigRobotSrc/plan_arene.PNG)

The robot starts from the inital surface (**number 1**).

The robot has to collect the cubes and to build a tower of 5 cubes maximum in the construction zone (**number 4**).
The team earns a lot a point if it respects the construction plan shown on the wall (**number 7**).




## Sections ( How does it work ?)

This paragraph explains the differents conceptions's steps and all the parts needed to build a tower. Here is the general flow diagram :

![alt text](mechanical/2018/bigRobotSrc/flow_diagram.PNG)


The system doesn't need any color detection because all the stars of cubes are always in the same order in the same direction.


### The base

![alt text](mechanical/2018/bigRobotSrc/base_render.jpg)

This is the firt thing I'm going to talk about but it was the last designed part. Indeed, every part had to be designed to know the constraints and dimensions. Thoses informations where essentials to design the base.

After the design in Fusion 360 in the model environment, the design was imported to the CAM environment still in Fusion.


![alt text](mechanical/2018/bigRobotSrc/base_cam.PNG)

It was then machined with a CNC :

![alt text](mechanical/2018/bigRobotSrc/base_machining.jpg)

### The entry

To swallow the cubes, an entry was designed. It uses simple DC motor with paint roller bought at the *Brico*.

![alt text](mechanical/2018/bigRobotSrc/entry.PNG)

The entry swallows the first 3 cubes and then select the fourth cube between the 2 last by pushing it with the toothed rack. After that the robot do the same step with the last cube. The rack moves with a rail available on *RS component* :


**IGUS rail**:
 * *Rail* :Igus N Series, NS-01-17-300, Linear Guide Rail 17mm width 300mm length
 * *Carriage* : Igus Linear Guide Carriage NW-02-17, N

To detect when the cube is in the lift, the robot uses a limit switch available on *RS Component* :

**Limit switch** : Snap Action Limit Switch, Roller Lever, Thermoplastic, NC, 125V.

### The lift


### The cage

![alt text](mechanical/2018/bigRobotSrc/cage.jpg)

The floors can rotate thanks to a dynamixel and a bearing wheel. To place the bearing wheel on the rotating axis we insert the last floor into the others floors with the bearing wheel between them as shown in the next figure :

![alt text](mechanical/2018/bigRobotSrc/cage_assembly.PNG)


## External parts

Most of the part are home made (CNC, 3D printing) but some parts a bought on the market :

* **IGUS rail**
  * *Small one*
    * Rail : Igus N Series, NS-01-17-300, Linear Guide Rail 17mm width 300mm length
    * Carriage : Igus Linear Guide Carriage NW-02-17, N
  * *Big one* : 
    * Rail : Igus N Series, NS-01-27-300, Linear Guide Rail 27mm width 300mm length
    * Carriage : Igus Linear Guide Carriage NW-02-27, N
* **Lift transmission**
  * *Pitch (distance between teeth)* : 2.032 mm
  * *Pulley* : aluminium, Glass Filled PC Timing Belt Pulley, 6mm Belt Width x 2.032mm Pitch, 36 Tooth, Maximum Bore Dia. 5mm
  * *Timing belt* : RS Pro, Timing Belt, 315 Tooth, 640.08mm Length X 6mm Width

* **Limit switch**
  * Snap Action Limit Switch, Roller Lever, Thermoplastic, NC, 125V
  
  
  # Author
  
  Crappe Martin
  


