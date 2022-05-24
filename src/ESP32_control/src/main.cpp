#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <Servo.h>
#include <DRV8833_MCPWM.h>
#include <drive.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <WiFi.h>

bool debug = true;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

const int servo1_pin = 32;
const int servo2_pin = 33;
const int servo3_pin = 25;
const int servo4_pin = 26;
const int servo5_pin = 27;
const int servo6_pin = 14;
const int servo7_pin = 12;

const int fl1 = 23;
const int fl2 = 19;
const int fr1 = 0;
const int fr2 = 2;
const int bl1 = 18;
const int bl2 = 5;
const int br1 = 4;
const int br2 = 16;
const int sleepl = 17;
const int sleepr = 15;

//extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
DRV8833 DRV8833_L;
DRV8833 DRV8833_R;
Drive drive;

rcl_timer_t timer;
rcl_subscription_t joint_state_subscriber, cmd_vel_subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
sensor_msgs__msg__JointState joint_state_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

bool micro_ros_init_successful;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK) && (debug == true)){return Serial.printf("Failed at line %d\n",__LINE__);}}

int rad2deg(double radian)
{
  int degree = static_cast<int>(radian * (180 / 3.14159));
  return degree+90;
}

void joint_states_cb(const void *msgin)
{
  const sensor_msgs__msg__JointState * joint_states_msg = (const sensor_msgs__msg__JointState *)msgin;
  servo1.write(rad2deg(joint_states_msg->position.data[4]));
  servo2.write(rad2deg(joint_states_msg->position.data[5]));
  servo3.write(180-rad2deg(joint_states_msg->position.data[6]));
  servo4.write(rad2deg(joint_states_msg->position.data[7]));
  servo5.write(rad2deg(joint_states_msg->position.data[8]));
  servo6.write(rad2deg(joint_states_msg->position.data[9]));

}

void cmd_vel_cb(const void *msgin)
{
  const geometry_msgs__msg__Twist * cmd_vel_msg = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(sleepl,HIGH);
  digitalWrite(sleepr,HIGH);
  drive.drive(cmd_vel_msg->linear.x,cmd_vel_msg->linear.y,cmd_vel_msg->angular.z);
}

void timer_cb(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) 
  {
    
  }
}


bool create_entities()
{	
	allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  // create subscriber
  rclc_subscription_init_default(&joint_state_subscriber, 
                                &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                "/joint_states");

  rclc_subscription_init_default(&cmd_vel_subscriber,
                                &node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                "/cmd_vel");

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_cb));

	// create executor
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 3, &allocator);
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &joint_state_msg, &joint_states_cb, ON_NEW_DATA));
  

	micro_ros_init_successful = true;
  if (debug == true)
  {
    Serial.println("Publishers & Subscribers are initialized");
  }
}


void setup()
{
  //init_i2c();
  set_microros_transports();
  micro_ros_init_successful = false;

  if (debug == true)
  {
    Serial.begin(115200);
  }

  pinMode(sleepl,OUTPUT);
  pinMode(sleepr,OUTPUT);

  pinMode(fl1,OUTPUT);
  pinMode(fl2,OUTPUT);
  pinMode(bl1,OUTPUT);
  pinMode(bl2,OUTPUT);

  pinMode(fr1,OUTPUT);
  pinMode(fr2,OUTPUT);
  pinMode(br1,OUTPUT);
  pinMode(br2,OUTPUT);

  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
  servo5.attach(servo5_pin);
  servo6.attach(servo6_pin);
  servo7.attach(servo7_pin);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  servo6.write(90);
  servo7.write(90);
  
  //    Wheels
  // L  Front  R
  // 0,0    1,0
  // 0,1    1,1
  DRV8833_L.attach(0,fl1,fl2,bl1,bl2,sleepl); // Driver 1, MCPWM Unit 0, Timer 0 & 1
  DRV8833_R.attach(1,fr1,fr2,br1,br2,sleepr); // Driver 2, MCPWM Unit 1, Timer 0 & 1
  drive.attach(DRV8833_L,DRV8833_R);

  // if (RMW_RET_OK != rmw_uros_sync_session(500) && debug == true)
  // {
  //   Serial.println("Time failed to sync");
  // }

}

void destroy_entities()
{
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_subscription_fini(&joint_state_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}
void loop() 
{
    static unsigned long long prev_connect_test_time;
    // check if the agent got disconnected at 10Hz
    if(micros() - prev_connect_test_time > 50000)
    {
      prev_connect_test_time = micros();
      // check if the agent is connected
      if(RMW_RET_OK == rmw_uros_ping_agent(50, 2))
      {
        // reconnect if agent got disconnected or haven't at all
        if (!micro_ros_init_successful) 
        {
          create_entities();
        } 
      } 
      else if(micro_ros_init_successful)
      {
        // clean up micro-ROS components
        destroy_entities();
      }
    }
    
    if(micro_ros_init_successful)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}