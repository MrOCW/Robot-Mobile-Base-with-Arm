#include <Arduino.h>
#include <Servo.h>
#include <DRV8833_MCPWM.h>
#include <drive.h>
#include <ros.h>
#include <time.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
//#include <MPU9250.h>

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

DRV8833 DRV8833_L;
DRV8833 DRV8833_R;
Drive drive;
bool check_zero_speeds(float speeds[])
{
  for (int speed = 0; speed<4; speed++)
  {
    if (speeds[speed] != 0)
    {
      return false;
    }
  }
  return true;     
}
int rad2deg(double radian)
{
  int degree = static_cast<int>(radian * (180 / 3.14159));
  return degree+90;
}

void joint_states_cb(const sensor_msgs::JointState &joint_states_msg)
{
  servo1.write(rad2deg(joint_states_msg.position[4]));
  servo2.write(rad2deg(joint_states_msg.position[5]));
  servo3.write(180-rad2deg(joint_states_msg.position[6]));
  servo4.write(rad2deg(joint_states_msg.position[7]));
  servo5.write(rad2deg(joint_states_msg.position[8]));
  servo6.write(rad2deg(joint_states_msg.position[9]));

}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg)
{
  digitalWrite(sleepl,HIGH);
  digitalWrite(sleepr,HIGH);
  drive.drive(cmd_vel_msg.linear.x,cmd_vel_msg.linear.y,cmd_vel_msg.angular.z);
}
//MPU9250 IMU(i2c0,0x68);

ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::JointState> joint_sub("/joint_states", joint_states_cb );
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", cmd_vel_cb );
//sensor_msgs::Imu imu_raw;
//ros::Publisher imu_pub("/imu/data_raw", &imu_raw);
//ros::Time ros_time;

void setup()
{
  int status = IMU.begin();
  if (debug == true)
  {
    Serial.begin(115200);
    // if (status < 0)
    // {
    //   Serial.println("IMU initialization unsuccessful");
    //   Serial.println("Check IMU wiring or try cycling power");
    //   Serial.print("Status: ");
    //   Serial.println(status);
    // }
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

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(joint_sub);
  nh.subscribe(cmd_vel_sub);
  // imu_raw.header.frame_id = "MPU9250_fl";
  // nh.advertise(imu_pub);
  //    Wheels
  // L  Front  R
  // 0,0    1,0
  // 0,1    1,1
  DRV8833_L.attach(0,fl1,fl2,bl1,bl2,sleepl); // Driver 1, MCPWM Unit 0, Timer 0 & 1
  DRV8833_R.attach(1,fr1,fr2,br1,br2,sleepr); // Driver 2, MCPWM Unit 1, Timer 0 & 1
  drive.attach(DRV8833_L,DRV8833_R);

  //   // setting the accelerometer full scale range to +/-8G
  // IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // // setting the gyroscope full scale range to +/-500 deg/s
  // IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // // setting DLPF bandwidth to 20 Hz
  // IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // // setting SRD to 19 for a 50 Hz update rate
  // IMU.setSrd(19);
  // IMU.calibrateAccel();
  // IMU.calibrateMag();
}

void loop()
{
  // // read the sensor
  // ros_time = nh.now();
  // /*if (check_zero_speeds(drive.speeds))
  // { // might cause further drift due to ignoring deceleration?
  //   imu_raw.linear_acceleration.x = 0;
  //   imu_raw.linear_acceleration.y = 0;
  //   imu_raw.linear_acceleration.z = 9.80665f; // set to 9.81 since we are dealing with 2D only
  //   imu_raw.angular_velocity.x = 0;
  //   imu_raw.angular_velocity.y = 0;
  //   imu_raw.angular_velocity.z = 0;
  //   imu_raw.header.stamp.sec = ros_time.sec;
  //   imu_raw.header.stamp.nsec = ros_time.nsec;
  //   imu_raw.orientation_covariance[0] = -1;
  // }
  // else
  // {*/
  // IMU.readSensor();
  // imu_raw.linear_acceleration.x = -IMU.getAccelX_mss();
  // imu_raw.linear_acceleration.y = IMU.getAccelY_mss();
  // imu_raw.linear_acceleration.z = 9.80665f; // set to 9.81 since we are dealing with 2D only
  // imu_raw.angular_velocity.x = IMU.getGyroX_rads();
  // imu_raw.angular_velocity.y = IMU.getGyroY_rads();
  // imu_raw.angular_velocity.z = -IMU.getGyroZ_rads();
  // imu_raw.header.stamp.sec = ros_time.sec;
  // imu_raw.header.stamp.nsec = ros_time.nsec;
  // imu_raw.orientation_covariance[0] = -1;
  // //}
  
  
  // if (debug==true)
  // {
  //   Serial.printf("Accel(%.6lf, %.6lf, %.6lf) Gyro(%.6lf, %.6lf, %.6lf) Mag(%.6f, %.6f, %.6f), Yaw: %6.f\n",\
  //   IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(),\
  //   IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),\
  //   IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT(),\
  //   float(atan2(IMU.getMagY_uT(), IMU.getMagX_uT())) * RAD_TO_DEG);
  // }

  // imu_pub.publish(&imu_raw);
  nh.spinOnce();
  delay(1);
}



