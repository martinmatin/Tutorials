# ROS Packages
ROS software is organized into packages, each of which contains some combination of code, data, and documentation. Usually there is a package for each 'function' of your robot. For example, you can have a package for navigation or for sensor capturing.
Thanks to the contributing community, there are many pre built packages available to download for free.

Let's create our first package!

* Creating a packages
```
cd ~/catkin_ws/src
catkin_create_pkg my_awesome_code rospy
```
This creates a new package named **my_awesome_code**. Inside that folder we'll find a **/src** directory where we will put our python code.

## Complementary info
### roscore
The `roscore` command is a service that provides connection information to nodes so that they can transmit messages to one another. **It is necessary to run this command before launching any package**.

### rosrun
To run a package, we use the `rosrun` command executed like this:
```
rosrun PACKAGE EXECUTABLE [ARGS]
```
**PACKAGE** is the name of the created package, for example **my_awesome_code**, and **EXECUTABLE** is the name of the python file containing the code. **Attention!** The python file needs to be executable, juste use `sudo chmod +x file.py`
