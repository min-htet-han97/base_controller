// Min Htet Han (ROM Robotics)
// minhtethan94@gmail.com                                

#include <math.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>

#include <avr/interrupt.h>
#include <avr/io.h>
MPU6050 mpu6050(Wire);

#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#define encoder0PinA 2
#define encoder0PinB 8
#define encoder1PinA 3
#define encoder1PinB 4
#define MotorPWM 5
#define MotorPWM2 9

#define MotorPin1 7
#define MotorPin2 6
#define MotorPin11 10
#define MotorPin22 11


 long     counts  = 0; 
 long     counts2 = 0;
double    actual_angleDegree,actual_angleDegree2, prv_actual_angleDegree,prv_actual_angleDegree2, actual_angleRevolution,actual_angleRevolution2;
double    desired_speedRPM,desired_speedRPM2, actual_speedRPM,actual_speedRPM2, actual_speedDegreeperSec,actual_speedDegreeperSec2;
double    now_time, samp_time, prv_time, Vsupply;
double    now_err,now_err2, K, PWM,PWM2, prv_err,prv_err2, Volt,Volt2, UI, prv_UI,tau,fc,now_x,now_x2,now_y,now_y2,prv_y,prv_y2;
double    Vi,Vi2,prv_Vi,prv_Vi2,Ki,Kd,Vd,Vd2;
double    desired_speedRADs,desired_speedRADs2,des_rad_m,des_rad_m2,base_width,wheel_dia  ;
double    angle,vel,vel2,vel_delta,new_vel,xinitial,yinitial,pose_x,pose_y,prv_pose_x,prv_pose_y;
double x ;
double z;

                                         
void setupPins();

void onTwist(const geometry_msgs::Twist &msg);

ros::NodeHandle node;
std_msgs::Float64 act_rpm;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
ros::Publisher pub1("mhh_arduino", &act_rpm);
ros::Subscriber<geometry_msgs::Twist> sub1("/cmd_vel", &onTwist); // keyboard input subscriber
//std_msgs::Int16 lwheel_msg;
//std_msgs::Int16 rwheel_msg;


//ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
//ros::Publisher rwheel_pub("rwheel", &rwheel_msg);
char base_link[]="/base_link";
char odom[]="/odom";

                                         
void setup()
{ Serial.begin(115200);
   setupPins();
   Wire.begin();
   mpu6050.begin();
  
  
  node.initNode();
  node.subscribe(sub1);
  node.advertise(pub1);
  broadcaster.init(node);
    
  K =0.2251;
  Ki = 0.401052;
  Kd = 0.002625;
  fc            = 1.0;
  base_width    = 0.3048;
  wheel_dia     = 0.065;
  xinitial      = 0.0;
  yinitial      = 0.0;
  pose_x        = 0.0;
  pose_y        = 0.0;
  tau           = 1/(2*PI*fc);
  mpu6050.calcGyroOffsets(true);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(MotorPWM, OUTPUT);
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(MotorPWM2, OUTPUT);
  pinMode(MotorPin11, OUTPUT);
  pinMode(MotorPin22, OUTPUT);
  digitalWrite(encoder0PinA, LOW);
  digitalWrite(encoder0PinB, LOW);
  digitalWrite(encoder1PinA, LOW);
  digitalWrite(encoder1PinB, LOW);
  attachInterrupt(digitalPinToInterrupt(2), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), readEncoder2, CHANGE);
}
void readEncoder() {
  if (digitalRead(encoder0PinB) == digitalRead(encoder0PinA)) {
    counts = counts - 1;
  }
  else                                                        {
    counts = counts + 1;
  }
}

void readEncoder2() {
  if (digitalRead(encoder1PinB) == digitalRead(encoder1PinA)) {
    counts2 = counts2 + 1;
  }
  else                                                        {
    counts2 = counts2 - 1;
  }
}



 

void setupPins()
        
{
  stop();
}

                                          
void stop()
{
  digitalWrite(MotorPin11, 0);
  digitalWrite(MotorPin22, 0);
  digitalWrite(MotorPin1, 0);
  digitalWrite(MotorPin2, 0);
  analogWrite(MotorPWM,   0);
  analogWrite(MotorPWM2,  0);
}

                                          

void onTwist(const geometry_msgs::Twist &msg)
{ 
  
     x = msg.linear.x;
     z = msg.angular.z;                                       
     desired_speedRADs  = ((2 * x) + (z * base_width))/( 2 * (wheel_dia/2));
     desired_speedRADs2 =  ((2 * x) + (-1.0 * z * base_width))/(2 * (wheel_dia/2));
     des_rad_m          =  (desired_speedRADs * 60);
     des_rad_m2         =  (desired_speedRADs2 * 60);

     desired_speedRPM    = des_rad_m / (2 * PI);
     desired_speedRPM2   = des_rad_m2 / (2 * PI);

}                                          
void loop()
{  

  now_time                 = micros()/1000000.0;// micros
  samp_time                = now_time - prv_time;//t(n)-t(n-1)
  mpu6050.update();
  
  t.header.frame_id = odom;
  t.child_frame_id  = base_link;
  
  angle = mpu6050.getAngleZ();
  vel     = ((wheel_dia/2) * actual_speedDegreeperSec * PI) / 180;
  vel2    = ((wheel_dia/2) * actual_speedDegreeperSec2 * PI) / 180;
  vel_delta = (vel - vel2);
  new_vel   = (vel + vel2)/2;
 
  pose_x    = xinitial  + (samp_time * new_vel * cos(angle * (PI/180)) + prv_pose_x);
  pose_y    = yinitial  + (samp_time * new_vel * sin(angle * (PI/180)) + prv_pose_y);
  
 
  
    
  actual_angleDegree       = ((360.0 * 1.0) / (660.0)) * counts;
  actual_angleDegree2       = ((360.0 * 1.0) / (660.0)) * counts2;//(64*70)) * counts; // degree 
   
  actual_angleRevolution   = actual_angleDegree / 360.0; 
  actual_angleRevolution2   = actual_angleDegree2 / 360.0; // revolution
  
  actual_speedDegreeperSec = (actual_angleDegree - prv_actual_angleDegree) / samp_time;
  actual_speedDegreeperSec2 = (actual_angleDegree2 - prv_actual_angleDegree2) / samp_time;

  now_x         = actual_speedDegreeperSec;
  now_x2         = actual_speedDegreeperSec2;
  
  now_y         = ( samp_time  * now_x + (tau * prv_y) ) / ( samp_time  + tau);
  now_y2         = ( samp_time  * now_x2 + (tau * prv_y2) ) / ( samp_time  + tau);
  
  

  
  actual_speedRPM          =  now_y   * (1 / 360.0) * (60); //
  actual_speedRPM2         =  now_y2   * (1 / 360.0) * (60);
  
  //desired_speedRPM         = -30;//90.0*sin(2*PI*0.5*now_time); // revolution per minute

  
  now_err                  = desired_speedRPM -  actual_speedRPM;
  now_err2                 = desired_speedRPM2 -  actual_speedRPM2;
  
  Vi                       = (Ki*samp_time * now_err)  +  prv_Vi;
  Vi2                      = (Ki*samp_time * now_err2)  +  prv_Vi2;
  
  Vd                       = Kd * (now_err - prv_err)/samp_time;
  Vd2                      = Kd * (now_err2 - prv_err2)/samp_time;
  
  Volt                     = (K * now_err) + Vi + Vd;
  Volt2                    = (K * now_err2) + Vi2 + Vd2;
  
  PWM                      = (Volt / 10) * 255;
  PWM2                     = (Volt2 / 10) * 255;
  
  PWM                      = abs(PWM);
  PWM2                     = abs(PWM2);

  
  if ( PWM > 255 ) {
    PWM = 255;
  }
  else             {
    PWM = PWM;
  }
 
    if ( PWM2 > 255 ) {
    PWM2 = 255;
  }
  else             {
    PWM2 = PWM2;
  }

  if (Volt > 0)  {
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);
//    digitalWrite(MotorPin11, LOW);
//    digitalWrite(MotorPin22, HIGH);
  }
  else if (Volt < 0)  {
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
//    digitalWrite(MotorPin11, HIGH);
//    digitalWrite(MotorPin22, LOW);
  }
  if (Volt2 > 0)  {
//    digitalWrite(MotorPin1, LOW);
//    digitalWrite(MotorPin2, HIGH);
    digitalWrite(MotorPin11, LOW);
    digitalWrite(MotorPin22, HIGH);
  }
  else if (Volt2 < 0)  {
//    digitalWrite(MotorPin1, HIGH);
//    digitalWrite(MotorPin2, LOW);
    digitalWrite(MotorPin11, HIGH);
    digitalWrite(MotorPin22, LOW);
  }

  analogWrite (MotorPWM, PWM);
  analogWrite (MotorPWM2, PWM2);//PWM
  

  
  t.transform.translation.x = pose_x;
  t.transform.translation.y = pose_y;
  t.transform.rotation = tf::createQuaternionFromYaw(angle*(PI/180.0));
  t.header.stamp= node.now();
  broadcaster.sendTransform(t);

  act_rpm.data = actual_speedRPM;
  pub1.publish(&act_rpm);
   node.spinOnce();
   delay(10);

   
  
  prv_actual_angleDegree = actual_angleDegree;
  prv_actual_angleDegree2= actual_angleDegree2;
  prv_time               = now_time;
  prv_err                = now_err;
  prv_err2               = now_err2;
  prv_y                  = now_y;
  prv_y2                 = now_y2;
  prv_Vi                 = Vi;
  prv_Vi2                = Vi2;
  prv_pose_x             = pose_x - xinitial;
  prv_pose_y             = pose_y - yinitial;
  
  
  
  Serial.print(counts2);  Serial.print(" ");
  Serial.println(counts ); Serial.print(" ");
  
}
