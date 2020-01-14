// /*
//  * motor_algorithm.h
//  *
//  *      Author: robotemperor
//  */
//pin information
#define motor_DIR1 26
#define motor_PWM1 12
//#define motor_ENA1 6
#define motor_ENA1 22 
#define motor_DIR2 19
#define motor_PWM2 13
#define motor_ENA2 9

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <pigpiod_if2.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

//custom header
#include <mobile_robot/motor_cmd.h>

class DcMotorForRaspberryPi
{
  public:

    DcMotorForRaspberryPi(int motor_dir, int motor_pwm, int motor_ena);
    void Motor_Controller(int direction, int pwm);
    // double Accel_Controller(bool direction, int desired_pwm);

    int pinum;
    int motor_ENA;
    int motor_DIR;
    int motor_PWM;
    int PWM_range;
    int PWM_frequency;
    int current_PWM;
    bool current_Direction;
    int acceleration;  

  private:
};

//ros communication

ros::Subscriber motor_first_command_sub;
ros::Subscriber motor_second_command_sub;

//function
void Initialize();
void Interrupt1(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Limit_Function(int pwm);

void motor_first_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg);
void motor_second_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg);
