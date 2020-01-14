/*
 * motor_algorithm.cpp
 *
 *      Author: robotemperor
 */
#include <mobile_robot/motor_algorithm.h>
#include <mobile_robot/motor_cmd.h>

    
DcMotorForRaspberryPi motor1 = DcMotorForRaspberryPi(motor_DIR1, motor_PWM1, motor_ENA1);
DcMotorForRaspberryPi motor2 = DcMotorForRaspberryPi(motor_DIR2, motor_PWM2, motor_ENA2);

using namespace std;
int PWM_limit;
void Interrupt1(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
volatile int EncoderCounter1;
volatile int EncoderCounter2;
bool switch_direction;
int Theta_Distance_Flag;

/////////////////////////////////////////////////////////////////////////////////////
void motor_first_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg)
{
  // motor1->speed_motor = msg->motor_desired_speed;
  // motor1->onoff       = msg->motor_onoff;
  // motor1.Motor_Controller(true, local_PWM);
  //motor1.Motor_Controller(true,msg->motor_desired_speed);
  motor1.Motor_Controller(msg->motor_desired_direction,msg->motor_desired_speed);
  //printf("motor1_speed : %f\n", msg->motor_desired_speed);
  //printf("motor1_dir : %d\n", msg->motor_desired_direction);
}
void motor_second_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg)
{
  // motor2->speed_motor = msg->motor_desired_speed;
  // motor2->onoff       = msg->motor_onoff;
  // motor2.Motor_Controller(true, local_PWM);
  //motor2.Motor_Controller(true,msg->motor_desired_speed);
  motor2.Motor_Controller(msg->motor_desired_direction,msg->motor_desired_speed);
  //printf("motor2_speed : %f\n", msg->motor_desired_speed);
  //printf("motor2_dir : %d\n", msg->motor_desired_direction);
}

/////////////////////////////////////////////////////////////////////////////////////
void Initialize()
{
  PWM_limit = 150;
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  callback(motor1.pinum, motor1.motor_ENA, FALLING_EDGE, Interrupt1);
  callback(motor2.pinum, motor2.motor_ENA, FALLING_EDGE, Interrupt2);

  switch_direction = true;
  Theta_Distance_Flag = 0;
  ROS_INFO("Initialize Complete");
}
void Interrupt1(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  EncoderCounter1 ++;
  //ROS_INFO("Interrupt1 is %d", EncoderCounter1);
}
void Interrupt2(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  EncoderCounter2 ++;
  //ROS_INFO("Interrupt2 is %d", EncoderCounter2);
}
int Limit_Function(int pwm)
{
  int output;
  if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)output = 0;
  else output = pwm;
  return output; 
}


/////////////////////////////////////////////////////////////////////////////////////
DcMotorForRaspberryPi::DcMotorForRaspberryPi(int motor_dir, int motor_pwm, int motor_ena)
{
  pinum=pigpio_start(NULL, NULL);
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
  }
  motor_DIR = motor_dir;
  motor_PWM = motor_pwm;
  motor_ENA = motor_ena;
  PWM_range = 512;
  PWM_frequency = 40000; 

  set_mode(pinum, motor_dir, PI_OUTPUT);
  //ROS_INFO("pinum is %d", pinum);
  //ROS_INFO("motor_dir is %d", motor_dir);
  set_mode(pinum, motor_pwm, PI_OUTPUT);
  set_mode(pinum, motor_ena, PI_INPUT);

  set_PWM_range(pinum, motor_pwm, PWM_range);
  set_PWM_frequency(pinum, motor_pwm, PWM_frequency);
  gpio_write(pinum, motor_DIR, PI_LOW);
  //ROS_INFO("pinum is %d", pinum);
  //ROS_INFO("motor_dir is %d", motor_dir);
  set_PWM_dutycycle(pinum, motor_PWM, 0);
  
  current_PWM = 0;
  current_Direction = true;
  acceleration = 5;
  ROS_INFO("Setup Fin");
}
void DcMotorForRaspberryPi::Motor_Controller(int direction, int pwm)
{
  if(direction == 1) //CW
  {
    gpio_write(pinum, motor_DIR, PI_LOW);
    set_PWM_dutycycle(pinum, motor_PWM, pwm);
    current_PWM = pwm;
    current_Direction = true;
  }
  else if(direction == -1) //CCW
  {
    gpio_write(pinum, motor_DIR, PI_HIGH);
    set_PWM_dutycycle(pinum, motor_PWM, pwm);
    current_PWM = pwm;
    current_Direction = false;
  }
}


int main (int argc, char **argv)
{
  printf("Motor node Start \n");
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Rate loop_rate(10);

  // std::string motor_first_topic = nh.param<std::string>("motor_first", "");
  // std::string motor_second_topic = nh.param<std::string>("motor_second", "");
  // motor_first_command_sub   = nh.subscribe(motor_first_topic, 1, motor_first_command_callback);
  // motor_second_command_sub  = nh.subscribe(motor_second_topic, 1, motor_second_command_callback);
  
  ros::Subscriber motor_first_command_sub  = nh.subscribe("/motor_1", 10, motor_first_command_callback);
  ros::Subscriber motor_second_command_sub = nh.subscribe("/motor_2", 10, motor_second_command_callback);
  // while(ros::ok())
  // {
  //   usleep(100);
  //   ros::spinOnce();
  // }
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  motor1.Motor_Controller(true, 0);
  motor2.Motor_Controller(true, 0);

  return 0;
}
