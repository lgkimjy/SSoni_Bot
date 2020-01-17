# SSoni_Bot

1.15
- ROS 와 PIGPIO를 이용한 DC모터 제어 (mobile_manager.cpp, motor_node.cpp)


1.14
- joystick 노드의 퍼블리싱 주기를 1000hz로 설정함 (joy_node.cpp에 autorepeat_rate를 바꿔주면 됨)
좌측 상단의 아날로그 스틱 데이터는 전진, 후진으로만 사용함
트리거버튼의 아날로그 데이터를 이용하여 모터의 속도를 조절할 수 있도록 함
