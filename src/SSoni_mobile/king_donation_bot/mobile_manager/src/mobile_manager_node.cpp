#include <mobile_manager/mobile_manager_node.h>
#include <mobile_manager/motor_cmd.h>

void initialize()
{
  gazebo_check = false;
  throttle_x = 0;
  throttle_y = 0;

  aileron_x = 0;
  aileron_y = 0;
  
  speed_ratio_rad = 0.0;
  max_speed = 100; // pwm
  rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);
  count = ros::Time::now();
  arm_displacement_msg.name = "left";
}
/////////////////////////    CALLBACK    /////////////////////////
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  ///// arm_displacement_prt ////
  if((ros::Time::now() - count).toSec() > 0.03)
  {
    if(fabs(msg->axes[4])>0.1)
    {
      arm_displacement_msg.pose.position.x = msg->axes[4]*0.001;
    }
    else
    {
      arm_displacement_msg.pose.position.x = 0;
    }
    if(fabs(msg->axes[3])>0.1)
    {
      arm_displacement_msg.pose.position.y = msg->axes[3]*0.001;
    }
    else
    {
      arm_displacement_msg.pose.position.y = 0;
    }

    arm_displacement_msg.pose.position.z = msg->axes[6]*0.001;

    rqyToQ = robotis_framework::convertRPYToQuaternion(0,0.01*msg->axes[7],0);
    arm_displacement_msg.pose.orientation.x = rqyToQ.x();
    arm_displacement_msg.pose.orientation.y = rqyToQ.y();
    arm_displacement_msg.pose.orientation.z = rqyToQ.z();
    arm_displacement_msg.pose.orientation.w = rqyToQ.w();

    arm_displacement_pub.publish(arm_displacement_msg);

    for(int idx = 0; idx <4; idx++)
    {
      if(msg->buttons[idx] == 1)
      {
        script_number_msg.data = idx+1;
        script_number_pub.publish(script_number_msg);
      }
    }
    count = ros::Time::now();
  }
  else
  {
    return;
  }
  ////  mobile_part  ////
  if(pow(msg->axes[0],2)+pow(msg->axes[1],2) > 0.01)
  {
    throttle_x = msg->axes[0];
    throttle_y = msg->axes[1];
  }
  else
  {
    throttle_x = 0;
    throttle_y = 0;
  }
  rotation_left  = msg->buttons[4];
  rotation_right = msg->buttons[5];
/*
  if(fabs(msg->axes[2]) == 1)
  {
    aileron_x = 0;
  }
  else
  {
    aileron_x = msg->axes[2];
  }

  if(fabs(msg->axes[5]) == 1)
  {
    aileron_x = 0;
  }
  else
  {
    aileron_x = msg->axes[5];
  }
*/

  aileron_x = msg->axes[2];
  aileron_y = msg->axes[5];

  //printf("sub ++ rotation_l : %d  || rotaiton_r : %d\n",rotation_left,rotation_right);
  if(rotation_left == 0 && rotation_right == 0)
  {
    motor_cmd_msg_1.motor_desired_speed = 0;
    motor_cmd_msg_2.motor_desired_speed = 0;
    rotation_left  = 0;
    rotation_right = 0;
  }
  if(msg->buttons[6] == 1)
  {
    enable_module_msg.data = "arm_module";
    enable_module_pub.publish(enable_module_msg);
    motor_cmd_msg_1.motor_desired_speed = 0;
    motor_cmd_msg_2.motor_desired_speed = 0;
    script_number_msg.data = 0;
    script_number_pub.publish(script_number_msg);
  }
  if(msg->buttons[7] == 1)
  {
    enable_module_msg.data = "action_module";
    enable_module_pub.publish(enable_module_msg);
    motor_cmd_msg_1.motor_desired_speed = 0;
    motor_cmd_msg_2.motor_desired_speed = 0;
    script_number_msg.data = 0;
    script_number_pub.publish(script_number_msg);
  }
}

void desired_vector_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if(pow(msg->position.x,2)+pow(msg->position.y,2) > 0.01)
  {
    throttle_x = msg->position.y;
    throttle_y = msg->position.x;
  }
  else
  {
    throttle_x = 0;
    throttle_y = 0;
  }
}

void arrivals_action_command_callback(const std_msgs::Int8::ConstPtr& msg)
{
  switch (msg->data)
  {
  case 0:
  {
    max_speed = 100;
    //rotation_left = 0;
    //rotation_right = 0;
  }
  break;
  case 4:
  {
    max_speed = 30;
    rotation_left = 1;
    rotation_right = 0;
  }
  break;
  case 5:
  {
    max_speed = 30;
    rotation_left = 0;
    rotation_right = 1;
  }
  break;
  case 1:
   {
     max_speed = 100;
     rotation_left = 0;
     rotation_right = 0;
     script_number_msg.data = 1;
     script_number_pub.publish(script_number_msg);
   }
  break;
  default:
  {
    max_speed = 100;
    rotation_left = 0;
    rotation_right = 0;
  }
  break;
  }
}
///////////////////////////////////////////////////////////////////////////
void wheel_direction_group(int8_t motor1, int8_t motor2)
{
  motor_cmd_msg_1.motor_desired_direction = motor1;
  motor_cmd_msg_2.motor_desired_direction = motor2;
}

void wheel_move_function(double x, double y, double ratio_l, double ratio_r)
{

  //direction classification
/*
  if(x < 0 && y > 0) //1사분면
  {
    speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
    wheel_direction_group(1,-1);
    motor_cmd_msg_1.motor_desired_speed = ((max_speed -(max_speed/(M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
  }
  else if(x > 0 && y > 0)      //2사분면
  {
    speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
    wheel_direction_group(1,-1);
    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = ((max_speed -(max_speed/(M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
  }
  else if(x > 0 && y < 0) //3사분면
  {
    speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
    wheel_direction_group(-1,1);
    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = ((max_speed -(max_speed/(M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
  }
  else if(x < 0 && y < 0) //4사분면
  {
    speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
    wheel_direction_group(-1,1);
    motor_cmd_msg_1.motor_desired_speed = ((max_speed -(max_speed/(M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
  }

*/  


  if(x < 0 && y > 0) // 1사분면
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,-1);

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;  
  }
  else if(x < 0 && y < 0) // 4 사분면
  {
    speed_ratio_rad = 0;
    wheel_direction_group(-1,1);

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;  
  }
  else if(x > 0 && y > 0) // 2 사분면
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,-1);

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;  
  }
  else if(x > 0 && y < 0) // 3 사분면
  {
    speed_ratio_rad = 0;
    wheel_direction_group(-1,1);

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;  
  }
  else if(x == 0 && y < 0) 
  {
    speed_ratio_rad = 0;
    wheel_direction_group(-1,1);

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;  
  }
  else if(x == 0 && y > 0) 
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,-1);

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;  
  }
  else // initial stop
  {
    motor_cmd_msg_1.motor_desired_speed = 0;
    motor_cmd_msg_2.motor_desired_speed = 0;
  }

  ratio_l += 1;
  ratio_r += 1;

  printf("ratio_l : %f\n", ratio_l);
  printf("ratio_r : %f\n", ratio_r);

  motor_cmd_msg_1.motor_desired_speed = fabs(motor_cmd_msg_1.motor_desired_speed * (ratio_l / 2));
  motor_cmd_msg_2.motor_desired_speed = fabs(motor_cmd_msg_2.motor_desired_speed * (ratio_r / 2));

/*
  if(fabs(x) < 0.5 && fabs(y) != 0)
  {   
    motor_cmd_msg_1.motor_desired_speed = max_speed*y;
    motor_cmd_msg_2.motor_desired_speed = -(max_speed*y);  
  }
  else if(fabs(x) != 0 && fabs(y) == 0)
  {   
    motor_cmd_msg_1.motor_desired_speed = -(max_speed*x);
    motor_cmd_msg_2.motor_desired_speed = 0;  
  }
  else if(fabs(x) == 0 && fabs(y) == 0)
  {   
    motor_cmd_msg_1.motor_desired_speed = 0;
    motor_cmd_msg_2.motor_desired_speed = 0;  
  }
  else
  {   
    motor_cmd_msg_1.motor_desired_speed = -(max_speed*x);
    motor_cmd_msg_2.motor_desired_speed = -(max_speed*y);  
  }
*/

  //speed decision
  // if(x == 0 || y==0)
  // {
  //   motor_cmd_msg_1.motor_desired_speed = max_speed*fabs(sqrt(pow(x,2)+pow(y,2)))*motor_cmd_msg_1.motor_desired_direction;
  //   motor_cmd_msg_2.motor_desired_speed = max_speed*fabs(sqrt(pow(x,2)+pow(y,2)))*motor_cmd_msg_2.motor_desired_direction;
  // }
}

void wheel_rotation(bool rotation_left, bool rotation_right)
{
  if(rotation_left || rotation_right)
  {
    if(rotation_left && rotation_right)//error
    {
      motor_cmd_msg_1.motor_desired_speed = 0;
      motor_cmd_msg_2.motor_desired_speed = 0;
      return;
    }
    if(rotation_left == 1)
    {
      wheel_direction_group(1,1);
      motor_cmd_msg_1.motor_desired_speed = fabs(max_speed*motor_cmd_msg_1.motor_desired_direction);
      motor_cmd_msg_2.motor_desired_speed = fabs(max_speed*motor_cmd_msg_2.motor_desired_direction);
    }
    if(rotation_right == 1)
    {
      wheel_direction_group(-1,-1);
      motor_cmd_msg_1.motor_desired_speed = fabs(max_speed*motor_cmd_msg_1.motor_desired_direction);
      motor_cmd_msg_2.motor_desired_speed = fabs(max_speed*motor_cmd_msg_2.motor_desired_direction);
    }
  }
  else
    return;
}

//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mobile_manager_node");
  ros::NodeHandle nh;
  initialize();

  nh.param("gazebo_check",  gazebo_check, false);

  motor1_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_1",10);
  motor2_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_2",10);

  arm_displacement_pub = nh.advertise<ssoni_arm_module_msgs::ArmCmd>("/heroehs/arm/displacement",1);
  script_number_pub = nh.advertise<std_msgs::Int32>("/heroehs/script_number",1);
  enable_module_pub = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  //ros::Subscriber motor_theta_dist_sub;
  ros::Subscriber joy_sub   = nh.subscribe("/joy", 1, joy_callback);
  ros::Subscriber desired_vector_sub = nh.subscribe("/ssoni/desired_vector", 1, desired_vector_callback);
  ros::Subscriber arrivals_action_command_sub = nh.subscribe("/ssoni/arrivals_action_command", 1, arrivals_action_command_callback);

  while(ros::ok())
  {
    wheel_move_function(throttle_x, throttle_y, aileron_x, aileron_y);
    //printf("rotation_l : %d  || rotaiton_r : %d\n",rotation_left,rotation_right);
    printf("th_x : %f  || th_y : %f\n",throttle_x,throttle_y);
    printf("al_x : %f  || al_y : %f\n",aileron_x,aileron_y);

    printf("motor1 : %f  || motor2 : %f\n",motor_cmd_msg_1.motor_desired_speed,motor_cmd_msg_2.motor_desired_speed);
    
    wheel_rotation(rotation_left, rotation_right);

    motor1_pub.publish(motor_cmd_msg_1);
    motor2_pub.publish(motor_cmd_msg_2);
    // if(gazebo_check)

    // {
    //   front_left_msg.data  =  (float) (motor_cmd_msg_2.motor_desired_speed*-1.0*0.05);
    //   front_right_msg.data =  (float) (motor_cmd_msg_1.motor_desired_speed*0.05);
    //   back_left_msg.data   =  (float) (motor_cmd_msg_4.motor_desired_speed*-1.0*0.05);
    //   back_right_msg.data  =  (float) (motor_cmd_msg_3.motor_desired_speed*0.05);

    //   front_left_pub.publish(front_left_msg);
    //   front_right_pub.publish(front_right_msg);
    //   back_left_pub.publish(back_left_msg);
    //   back_right_pub.publish(back_right_msg);
    // }
    usleep(1000);
    /*
    printf("---------------------------------------\n");
    printf("DIR Motor1 :: %d \n", motor_cmd_msg_1.motor_desired_direction);
    printf("DIR Motor2 :: %d \n", motor_cmd_msg_2.motor_desired_direction);
    printf("DIR Motor3 :: %d \n", motor_cmd_msg_3.motor_desired_direction);
    printf("DIR Motor4 :: %d \n", motor_cmd_msg_4.motor_desired_direction);
    printf("---------------------------------------\n");
    printf("2    %f    ", motor_cmd_msg_2.motor_desired_speed);
    printf("1    %f \n" , motor_cmd_msg_1.motor_desired_speed);
    printf("4    %f    ", motor_cmd_msg_4.motor_desired_speed);
    printf("3    %f \n" , motor_cmd_msg_3.motor_desired_speed);
     */
    ros::spinOnce();
  }
  return 0;
}
