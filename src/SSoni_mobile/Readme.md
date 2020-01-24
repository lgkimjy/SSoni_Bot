# SSoni_Bot

1.21
- xavier와 통신시 모터의 foward와 backward 시작 시 모터의 가감속이 이상한게 확인 됨

1.20
- Ssoni_Bot과 연결하여 모터 구동 완료, 방향제어 및 xavier와 통신하여 확인완료

1.15
- ROS 와 PIGPIO를 이용한 DC모터 제어 (mobile_manager.cpp, motor_node.cpp)


1.14
- joystick 노드의 퍼블리싱 주기를 1000hz로 설정함 (joy_node.cpp에 autorepeat_rate를 바꿔주면 됨)
- 좌측 상단의 아날로그 스틱 데이터는 전후진으로 사용
- 트리거버튼의 아날로그 데이터를 이용, 좌우릐 모터의 속도를 조절하여 조향 
