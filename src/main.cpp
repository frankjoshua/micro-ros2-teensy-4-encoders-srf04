#include <Arduino.h> 

#include "Kinematics.h"
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#include "Odometry.h"
#include <NewPing.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/empty.h>
#include <nav_msgs/msg/odometry.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#define EncoderCSLeft 6
#define EncoderCSRight 5
#define PID_MAX 2048
#define K_P 10.0   // P constant
#define K_I 0.05 // I constant
#define K_D 5.0  // D constant

#define SONAR_NUM 1     // Number of sensors.
#define MAX_DISTANCE 40 // Maximum distance (in cm) to ping.

//define your robot' specs here
#define TICKS_PER_REVOLUTION 64000 // Number of encoder ticks for full rotation
#define MAX_RPM 330                // motor's maximum RPM
#define WHEEL_DIAMETER 0.15        // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.35    // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30    // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN


rcl_publisher_t publisher;
rcl_publisher_t tf_publisher;
rcl_publisher_t range_publisher;
rcl_publisher_t vel_publisher;
rcl_subscription_t subscriber;
tf2_msgs__msg__TFMessage * tf_message;
nav_msgs__msg__Odometry * odom_message;
geometry_msgs__msg__Twist * vel_twist_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32 msg;
sensor_msgs__msg__Range range;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

NewPing sonar[SONAR_NUM] = {
    // Sensor object array.
    NewPing(4, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
};

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
PID pidLeft(-PID_MAX, PID_MAX, K_P, K_I, K_D);
PID pidRight(-PID_MAX, PID_MAX, K_P, K_I, K_D);
Encoder encoder(EncoderCSLeft, EncoderCSRight, WHEEL_DIAMETER / 2, TICKS_PER_REVOLUTION);
Motor leftMotor(2);
Motor rightMotor(1);
Odometry odometry;

char base_link[] = "/odom_wheel";
char odom[] = "/odom";
long lastCommandTime = 0;
long lastPingTime = 0;
Kinematics::rpm goalRPM;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

void error_loop(int errorCode){
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 
  Serial.begin(115200);
  long count = 0;
  while(1){
    count++;
    if(count % 5 == 0){
      Serial.print("Error code: ");
      Serial.println(errorCode);
    }  
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  // error_loop(111111);
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  lastCommandTime = millis();
  goalRPM = kinematics.getRPM(msg->linear.x, msg->linear.y, msg->angular.z);
}

void createROSNode(){
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
}

void setupROS(){
  

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &range_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "range"));

  RCCHECK(rclc_publisher_init_default(
    &vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "vel"));

  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));

  msg.data = 0;

  vel_twist_msg = geometry_msgs__msg__Twist__create();

  if(!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
      &tf_message,
      (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop(88888);
  }

  tf_message = tf2_msgs__msg__TFMessage__create();
geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 2);
  tf_message->transforms.size = 2;
  // tf_message->transforms.data[0].header.stamp.sec = 0;
  tf_message->transforms.data[0].header.frame_id =
    micro_ros_string_utilities_set(tf_message->transforms.data[0].header.frame_id, "/odom");
    tf_message->transforms.data[0].child_frame_id =
    micro_ros_string_utilities_set(tf_message->transforms.data[0].child_frame_id, "/base_link");
  // error_loop();
  tf_message->transforms.data[1].header.frame_id =
    micro_ros_string_utilities_set(tf_message->transforms.data[1].header.frame_id, "/odom");
     tf_message->transforms.data[1].child_frame_id =
    micro_ros_string_utilities_set(tf_message->transforms.data[1].child_frame_id, "/imu_odom");
  
}

void stopIfNoCommand(){
  if (millis() - lastCommandTime > 400)
  {
    goalRPM = kinematics.getRPM(0, 0, 0);
  }
}

void publishOdomTransform(){
  struct timespec tv = {0};
  clock_gettime(0, &tv);
  Encoder::RPM rpm = encoder.getRPM();
  leftMotor.adjust(pidLeft.compute(goalRPM.motor1, rpm.left));
  rightMotor.adjust(pidRight.compute(goalRPM.motor2, rpm.right));
  Kinematics::velocities vel = kinematics.getVelocities(rpm.left, rpm.right, 0, 0);
  Odometry::Position position = odometry.calculatePosition(vel.linear_x, vel.angular_z);
  tf_message->transforms.data[0].transform.translation.x = position.x;
  tf_message->transforms.data[0].transform.translation.y = position.y;
  tf_message->transforms.data[0].transform.rotation.z = sin(position.theta * 0.5);
  tf_message->transforms.data[0].transform.rotation.w = cos(position.theta * 0.5);
  tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
	tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;
  tf_message->transforms.data[1].header.stamp.nanosec = tv.tv_nsec;
	tf_message->transforms.data[1].header.stamp.sec = tv.tv_sec;

  vel_twist_msg->angular.z = vel.angular_z;
  vel_twist_msg->linear.x = vel.linear_x;
  // Debuging
  // vel_twist_msg->linear.z = goalRPM.motor1;
  // vel_twist_msg->linear.y = rpm.left;
  // vel_twist_msg->angular.y = pidLeft.compute(goalRPM.motor1, rpm.left);

  RCSOFTCHECK(rcl_publish(&tf_publisher, tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&vel_publisher, vel_twist_msg, NULL));
}

void publishRange(){
  if (millis() - lastPingTime > 50)
  {
    struct timespec tv = {0};
    clock_gettime(0, &tv);
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
      // delay(30); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      range.header.stamp.nanosec = tv.tv_nsec;
      range.header.stamp.sec = tv.tv_sec;
      range.min_range = 0.01;
      range.max_range = MAX_DISTANCE / 100;
      range.field_of_view = 0.261799;
      float cm = sonar[i].ping_cm();
      if (cm == 0)
      {
        range.range = INFINITY;
      }
      else
      {
        range.range = (float)cm / 100;
      }
      RCSOFTCHECK(rcl_publish(&range_publisher, &range, NULL));
    }
  }
}

void setup() {
  set_microros_transports();
  delay(2000);

  createROSNode();
  setupROS();

  // goalRPM = kinematics.getRPM(0.2, 0, 0);
}

void loop() {
  
  delay(10);
  stopIfNoCommand();
  publishOdomTransform();
  publishRange();
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}