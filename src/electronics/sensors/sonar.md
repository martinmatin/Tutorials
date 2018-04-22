
# Sonar #

This is a draft of the tutorial, not the last version so all the instructions are not here yet.

This tutorial explains the code to use and transfer sonar sensors information with ROS.

For the «Cortex» robot we used four sensors, one on each side so that’s what the final code presented at the end of this tutorial is based on.

Firstly, in this code we are going to include the same libraries as shown in the Arduino Publisher Tutorial so you can refer to that part for the explanations.
An additional useful library used for the sensors is the NewPing.h. So here are all the necessary libraries :
```
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>
```
Then you have to define the pins for each sensor. In this case, the ultrasound sensor has two pins (Trigger ans Echo). Here is an exemple with the sensor located on the right of the robot:
```
#define TRIGGER_PINR  5
#define ECHO_PINR    4   
```
You also have to specify the maximum distance at which you want the sensor to still be able to detect :
```
#define MAX_DISTANCE 300
```
Now as we are using the NewPing library you can simply create an ultrasound sensor object by doing this :
```
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
```
where you specify the pins and the maximum distance defined previously.

Moreover as mentioned in the Arduino Publisher Tutorial you specify the type of the ultrasound message and the name you want to assign to it :
```
sensor_msgs::Range range_msg_right;
```
You also have to add the following line which has also been explained in the Arduino Publisher Tutorial :
```
ros::Publisher pub_range4("ultrasound_right", &range_msg_right);
```
We then have to fill each sonar object with the initialisation information associated :
```
range_msg_right.radiation_type = sensor_msgs::Range::ULTRASOUND;
range_msg_right.header.frame_id =  "ultrasound_right";
range_msg_right.field_of_view = 0.3665;  // fake
range_msg_right.min_range = 0.0;
range_msg_right.max_range = MAX_DISTANCE;
```
Now we can add parts to the void loop of the Arduino code. The most important and useful one specifies the distance to an obstacle :
```
range_msg_right.range = tmp/100;
```
We can then publish that information about the distance (you can again refer to the Arduino Publisher Tutorial for the publishing part) :
```
pub_range4.publish(&range_msg_right);
```
Here is the whole code for four ultrasound sensors based on the one presented in this link :
<https://www.youtube.com/watch?v=gm3e-51ohgQ>

```
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

#define TRIGGER_PINR  5   //back
#define ECHO_PINR    4   

#define TRIGGER_PINL  7   //front
#define ECHO_PINL     6  

#define TRIGGER_PINB  12   //left
#define ECHO_PINB     13 

#define TRIGGER_PINF  11   //right
#define ECHO_PINF     10 

#define MAX_DISTANCE 300 // Maximum distance we want to ping  

NewPing sonarL(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // back us 
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE); // front us
NewPing sonarB(TRIGGER_PINB, ECHO_PINB, MAX_DISTANCE); // left us
NewPing sonarF(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE); // right us

ros::NodeHandle  nh;

sensor_msgs::Range range_msg_rear;
sensor_msgs::Range range_msg_front;
sensor_msgs::Range range_msg_left;
sensor_msgs::Range range_msg_right;
ros::Publisher pub_range1("ultrasound_rear", &range_msg_rear);
ros::Publisher pub_range2("ultrasound_front", &range_msg_front);
ros::Publisher pub_range3("ultrasound_left", &range_msg_left);
ros::Publisher pub_range4("ultrasound_right", &range_msg_right);
 

char frameid[] = "base_link";

long duration;
 float tmp;

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);

  range_msg_rear.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_rear.header.frame_id =  "ultrasound_rear";
  range_msg_rear.field_of_view = 0.3665;  // fake
  range_msg_rear.min_range = 0.0;
  range_msg_rear.max_range = MAX_DISTANCE;
  
  range_msg_front.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_front.header.frame_id =  "ultrasound_front";
  range_msg_front.field_of_view = 0.3665;  // fake
  range_msg_front.min_range = 0.0;
  range_msg_front.max_range = MAX_DISTANCE; 
   
  range_msg_left.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_left.header.frame_id =  "ultrasound_front";
  range_msg_left.field_of_view = 0.3665;  // fake
  range_msg_left.min_range = 0.0;
  range_msg_left.max_range = MAX_DISTANCE;  
  
  range_msg_right.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_right.header.frame_id =  "ultrasound_right";
  range_msg_right.field_of_view = 0.3665;  // fake
  range_msg_right.min_range = 0.0;
  range_msg_right.max_range = MAX_DISTANCE;
}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    tmp=sonarL.ping_cm();
    range_msg_rear.range = tmp/100;
    range_msg_rear.header.stamp = nh.now();
    pub_range1.publish(&range_msg_rear);

    tmp=sonarR.ping_cm();
    range_msg_front.range = tmp/100;
    range_msg_front.header.stamp = nh.now();
    pub_range2.publish(&range_msg_front);
    
     tmp=sonarB.ping_cm();
    range_msg_left.range = tmp/100;
    range_msg_left.header.stamp = nh.now();
    pub_range2.publish(&range_msg_left);

    tmp=sonarF.ping_cm();
    range_msg_right.range = tmp/100;
    range_msg_right.header.stamp = nh.now();
    pub_range4.publish(&range_msg_right);

    range_time =  millis() + 50;
  }
  nh.spinOnce();
}


```
