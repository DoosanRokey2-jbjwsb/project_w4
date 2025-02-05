![image](https://github.com/user-attachments/assets/32f21dad-5f3a-4ba2-b221-0e1c7b858ae0)

<250204>
1. Gui node (QT)
	<MainWindows>
	- camera 창 (구현 미완: 카메라 영상 받아오는 문제)
	- 버튼: json에 값을 담아서 토픽 발행
		1) 이동 (구현 완료)
			- move_button_clicked() 함수
		2) 정지 (구현 완료)
			- stop_button_clicked() 함수
	- 상태 정보창 (구현 완료)
	<Node>
		1) gui 스레드
			- PyQt 이벤트 루프 처리
			- GUI 관련 작업: 버튼 클릭 이벤트, 화면 갱신
		2) ros 스레드: ros2 메시지 처리
			- 0.1초마다 한번씩 ROS2 메시지를 처리: spin_once()
			- subcription (conveyor/status)
			- publisher (conveyor/control)
			- 입력 받은 거리값(mm)을 토픽 발행해서 conveyor 노드로 넘겨줌: send_control_command()
2. Conveyor node
	- 멀티 스레드
		1) Node
			- subcription (conveyor/control)
			- publisher (conveyor/status)
			- gui 노드로부터 넘어온 입력받은 거리값(mm)을 serial.write로 아두이노로 전달
		2) Serial
			- serial_loop(): 
				상태('_', '.', 'DISCONNECTED')가 바뀔 때만 Status 발행 (토픽)
			- connect_serial(): 
				disconnect 후, 재 연결시 READY 상태 발행 (토픽)
				
	
