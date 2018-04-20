# Ros
ROS starts with the ROS Master. The Master allows all other ROS pieces of software (Nodes) to find and talk to each other. That way, we do not have to ever specifically state “Send this sensor data to that computer at 127.0.0.1. We can simply tell Node 1 to send messages to Node 2. But... how do Nodes do this? By publishing and subscribing to `TOPICS`.

The ROS Master provides name registration and lookup to the rest of the Computation Graph.

![alt text](http://www.clearpathrobotics.com/assets/guides/ros/_images/ros101one.png "ROS GRAPH")

> This content is based on [Clearpath Robotics](https://www.clearpathrobotics.com/) documentation. For further information refer to their [ROS Tutorials](http://www.clearpathrobotics.com/assets/guides/ros/Intro%20to%20the%20Robot%20Operating%20System.html).

# Nodes

Nodes are executables that can communicate with other processes using topics,
services, or the Parameter Server. Using nodes in ROS provides us with fault
tolerance and separates the code and functionalities, making the system simpler.

A node must have a unique name in the system. This name is used to permit the
node to communicate with another node using its name without ambiguity. 

ROS has tools to handle nodes and give us information about it, such as `rosnode`.

The rosnode tool is a command-line tool used to display information about nodes,
such as listing the currently running nodes. The supported commands are as follows:
* `rosnode info NODE`: This prints information about a node
* `rosnode kill NODE`: This kills a running node or sends a given signal
* `rosnode list`: This lists the active nodes
* `rosnode machine hostname`: This lists the nodes running on a particular machine or lists machines
* `rosnode ping NODE`: This tests the connectivity to the node.
* `rosnode cleanup`: This purges the registration information from unreachable nodes

# Topics
Topics are buses used by nodes to transmit data. Topics can be transmitted without a
direct connection between nodes, which means that the production and consumption
of data are decoupled. A topic can have various subscribers and can also have
various publishers, but you can take care about publishing the same topic with
different nodes because it can create conflicts.

Each topic is strongly typed by the ROS message type used to publish it, and nodes
can only receive messages from a matching type. A node can subscribe to a topic
only if it has the same message type.

ROS has a tool to work with topics called `rostopic`. It is a command-line tool
that gives us information about the topic or publishes data directly on the network.
This tool has the following parameters:
* `rostopic list`: This prints information about active topics.
* `rostopic pub /topic type args`: This publishes data to the topic.
It allows us to create and publish data in whatever topic we want,
directly from the command line.
* `rostopic echo /topic`: This prints messages to the screen.
* `rostopic find message_type`: This finds topics by their type.
* `rostopic info /topic`: This prints information about the active topic,
* `rostopic hz /topic`: This displays the publishing rate of the topic.
topics published, ones it is subscribed to, and services.
* `rostopic type /topic`: This prints the topic type, that is, the type of
message it publishes.
* `rostopic bw /topic`: This displays the bandwidth used by the topic.

# Messages
A node sends information to another node using messages that are published
by topics. The message has a simple structure that uses standard types or types
developed by the user.

Message types use the following standard ROS naming convention; the name of the
package, then /, and then the name of the .msg file. For example, **std_msgs/msg/String.msg** has the **std_msgs/String** message type.

ROS has the `rosmsg` command-line tool to get information about messages. The
accepted parameters are as follows:
* `rosmsg show`: This displays the fields of a message
* `rosmsg list`: This lists all messages
* `rosmsg package`: This lists all of the messages in a package
* `rosmsg packages`: This lists all of the packages that have the message
* `rosmsg users`: This searches for code files that use the message type

