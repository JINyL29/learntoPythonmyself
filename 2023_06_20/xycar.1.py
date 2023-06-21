#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultrasonicData = [0.0, 0.0, 0.0, 0.0, 0.0]  # 초음파데이터컨테이너.[좌 좌향각 전 우향각 우]
SIDE_MAX = 300  # 양옆 최대 길이
SIDE_MIN = 130  # 양옆 최소 길이
FRONT_LIMIT = 340  # 앞쪽 초음파 길이제한
CORNER_ANGLE = 44  # 코너 도는 각도
AVOID_ANGLE = 5  # 장애물 피할 때의 각도 설정


def callback(msg):  # msg를 받는 callback 함수 정의
    global ultrasonicData  # ultrasonicData를 전역변수(global)로 설정
    ultrasonicData = msg.data # msg.data값을 ultrasonicData에 대입


rospy.init_node('driver')  # rospy에 있는 'driver'저장파일 초기화
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)  # callback함수에 저장된 초음파데이터 ultrasonic을 Subscriber로 데이터를 받아옴
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)  # xycar_motor데이터를 Publisher가 motor_pub로 데이터 전송

xycar_msg = xycar_motor()  # xycar_motor 데이터가 xycar_msg에 대입

while not rospy.is_shutdown():       # rospy가 중지되지 않으면
    L_DIS = ultrasonicData[0]        # ultrasonicData 내부 컨테이너 0번인 데이터값이 L_DIS에 대입
    R_DIS = ultrasonicData[4]        # ultrasonicData 내부 컨테이너 4번인 데이터값이 R_DIS에 대입
    DIA_L_DIS = ultrasonicData[1]    # ultrasonicData 내부 컨테이너 1번인 데이터값이 DIA_L_DIS에 대입
    DIA_R_DIS = ultrasonicData[3]    # ultrasonicData 내부 컨테이너 3번인 데이터값이 DIA_R_DIS에 대입
    FRONT_DIS = ultrasonicData[2]    # ultrasonicData 내부 컨테이너 2번인 데이터값이 FRONT_DIS에 대입
    angle = 0                        # angle에 0값 대입
    speed = 50                       # speed에 50값 대입

    if FRONT_DIS < FRONT_LIMIT and DIA_R_DIS > 155:                       # 초음파로 받은 데이터 값 중 앞쪽 초음파가 앞쪽에 제한거리를 둔 데이터값보다 작고, 우향각초음파로부터 받은 데이터값이 155를 넘어갈 때,
        angle = CORNER_ANGLE                                              # CORNER_ANGLE값을 angle에 대입 후 xycar_msg.angle에 데이터값을 대입하여 xycar가 CORNER_ANGLE값으로 돌도록 한다.
        if DIA_L_DIS > DIA_R_DIS:                                         # 도는 도중, 좌향각초음파 데이터값이 우향각초음파 데이터값보다 크다면,
            angle = -CORNER_ANGLE                                         # -CORNER_ANGLE값을 angle에 대입 후 xycar_msg.angle에 데이터값을 대입하여 xycar가 CORNER_ANGLE값으로 돌도록 한다.

        if FRONT_DIS < FRONT_LIMIT and DIA_R_DIS > SIDE_MAX:              # 위의 상황을 기반으로, 초음파로 받은 데이터 값 중 앞쪽 초음파가 앞쪽에 제한거리를 둔 데이터값보다 작고, 양옆 최대 길이보다 초음파데이터로 받은 우향각 거리가 크다면,
            angle = CORNER_ANGLE                                          # CORNER_ANGLE값을 angle에 대입 후 xycar_msg.angle에 데이터값을 대입하여 xycar가 CORNER_ANGLE값으로 돌도록 한다. 커브를 돌고 난 후 바로 직면하는 장애물을 피하기 위한 코드.
            while not math.isclose(R_DIS + L_DIS, 192):                   # R_DIS + L_DIS와 초음파 데이터값 192가 같지 않다면,
                # 기다림
                while ultrasonicData == [0.0, 0.0, 0.0, 0.0, 0.0]:        # wultrasonicData가 [0.0, 0.0, 0.0, 0.0, 0.0]일 때,
                    rospy.sleep(0.01)                                     # 0.01초 지연

                # ultrasonicData 값 업데이트
                L_DIS = ultrasonicData[0]
                R_DIS = ultrasonicData[4]
                DIA_L_DIS = ultrasonicData[1]
                DIA_R_DIS = ultrasonicData[3]
                FRONT_DIS = ultrasonicData[2]

                xycar_msg.angle = angle
                xycar_msg.speed = speed
                motor_pub.publish(xycar_msg)
                rospy.sleep(0.01)

                if FRONT_DIS < 230:                                       # 초음파로 받은 앞쪽 거리가 230보다 작을때
                    angle = 0                                             # 돌지않는다.
                    break


    elif FRONT_DIS < FRONT_LIMIT and DIA_L_DIS > 155:                     # 초음파로 받은 데이터 값 중 앞쪽 초음파가 앞쪽에 제한거리를 둔 데이터값보다 작고, 좌향각초음파로부터 받은 데이터값이 155를 넘어갈 때,
        angle = -CORNER_ANGLE                                             # CORNER_ANGLE값을 angle에 대입 후 xycar_msg.angle에 데이터값을 대입하여 xycar가 CORNER_ANGLE값으로 돌도록 한다.
        if DIA_R_DIS > DIA_L_DIS:                                         # 도는 도중, 우향각초음파 데이터값이 좌향각초음파 데이터값보다 크다면,
            angle = CORNER_ANGLE                                          # -CORNER_ANGLE값을 angle에 대입 후 xycar_msg.angle에 데이터값을 대입하여 xycar가 CORNER_ANGLE값으로 돌도록 한다.

        if FRONT_DIS < FRONT_LIMIT and DIA_L_DIS > SIDE_MAX:              # 위의 상황을 기반으로, 초음파로 받은 데이터 값 중 앞쪽 초음파가 앞쪽에 제한거리를 둔 데이터값보다 작고, 양옆 최대 길이보다 초음파데이터로 받은 좌향각 거리가 크다면,
            angle = -CORNER_ANGLE                                         # CORNER_ANGLE값을 angle에 대입 후 xycar_msg.angle에 데이터값을 대입하여 xycar가 CORNER_ANGLE값으로 돌도록 한다. 커브를 돌고 난 후 바로 직면하는 장애물을 피하기 위한 코드.
            while not math.isclose(R_DIS + L_DIS, 192):                   # R_DIS + L_DIS와 초음파 데이터값 192가 같지 않다면,
                # 기다림
                while ultrasonicData == [0.0, 0.0, 0.0, 0.0, 0.0]:        # wultrasonicData가 [0.0, 0.0, 0.0, 0.0, 0.0]일 때,
                    rospy.sleep(0.01)                                     # 0.01초 지연

                # ultrasonicData 값 업데이트
                L_DIS = ultrasonicData[0]
                R_DIS = ultrasonicData[4]
                DIA_L_DIS = ultrasonicData[1]
                DIA_R_DIS = ultrasonicData[3]
                FRONT_DIS = ultrasonicData[2]

                xycar_msg.angle = angle
                xycar_msg.speed = speed
                motor_pub.publish(xycar_msg)
                rospy.sleep(0.01)

                if FRONT_DIS < 230:
                    angle = 0
                    break

    else:                                                                 # 앞선 두 가지 상황이 아닐경우,
        angle = 0                                                         # 돌지 않는다.

    if angle == 0:                                                        # 도는 각도 테이터 값이 0일 때,
        if DIA_L_DIS < SIDE_MIN:                                          # 좌향각초음파로부터 받은 데이터값이 양옆 최소 거리 데이터값보다 작을 경우,
            angle = AVOID_ANGLE                                           # 장애물 피할 때의 각도만큼 돈다.
        elif DIA_R_DIS < SIDE_MIN:                                        # 우향각초음파로부터 받은 데이터값이 양옆 최소 거리 데이터값보다 작을 경우,
            angle = -AVOID_ANGLE                                          # 장애물 피할 때의 각도의 반대 방향으로 돈다.

    xycar_msg.angle = angle                                               # angle값을 xycar_msg.angle에 대입
    xycar_msg.speed = speed                                               # speed값을 xycar_msg.speed에 대입
    motor_pub.publish(xycar_msg)                                          # motor_pub이 xycar_msg데이터값을 전송