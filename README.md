국제 전기차 엑스포에서 주최한 제 2회 국제 대학생 전기차 자율주행 경진대회 1/10 자율주행 종목에 출전했던 부산대학교 기계공학부 팀의 코드입니다. IEVE 제 2회 국제 대학생 EV 자율주행 경진대회 1/10AA 종목 부산대학교 기계공학부팀 주행영상 https://youtu.be/jxaMWl2RJnI

HW 구성

동력 : 12V DC motor (YFROBOT GM25 12CPR)

조향 : servo motor (MG 996R)

센서 : CSI camera (RPi v2), 초음파 센서 (HC-SR04)

CPU : Jetson Nano 4GB B01

MCU : Arduino Mega 2560

motor driver : YFROBOT PM-R3

CPU to MCU 통신 : serial 연결

SW 구성

ros (ev_main) : 카메라를 읽은 후 조향값을 아두이노로 publish

rosserial

arduino : ros msg로 수동/자동 모드 변경 및 초음파 센서를 읽어 장애물 인식

ev_main library : opencv, gstreamer

arduino library : YFROBOT's motor library (MotorDriver.h), rosserial

조향 알고리즘

: 본 코드는 카메라가 차선을 정상적으로 보고 있을 때 차선이 화면 안으로 들어와서 나가는 점을 이용해 화면의 가장자리에서 차선을 인식하고 두 점을 이어 x[0] (아랫쪽에서 인식된 차선의 x값), x[1] (나머지 한쪽의 x값), theta(두 점이 y축과 이루는 각도)를 이용해 조향합니다.

Region Of Interest (reg_of_int) Yellow mask (main 함수에서 처리) Get displacement (get_disp) Get steering angle (get_steer) reg_of_int input : origianl image output : cropped image 처리 부하를 줄이기 위해 필요 없는 부분을 없앨 때 사용합니다. 본 코드는 가장자리만 보면 되기 때문에 상하좌우 5%의 픽셀만 남기고 가운데 부분을 없앴습니다.

yellow mask input : cropped image output : masked image (노란색이 추출된 binary image) 차선이 노란색이기에 노란색의 범위를 지정한 뒤 그 사이의 값을 가지는 픽셀에 1, 나머지 픽셀에 0을 넣어 반환합니다.

get_disp input : masked image output : x, theta, num_detected masked image에서 contour(덩어리)를 추출합니다. 조향 알고리즘의 의도대로 2개가 추출된 경우 x와 theta를 업데이트합니다. x는 카메라의 중앙이 0의 값을 가지며 theta는 y축을 기준으로 반시계방향으로 측정됩니다.

line detection by looking at edge (via opencv)

https://youtu.be/vgk8-RSu_DI

get_steer input : x, theta output : steering_angle x와 theta를 이용해 서보모터에 명령을 줄 값 steering_angle을 도출합니다.

theta에 의한 조향 steer algorithm explained theta에 정비례해 steer값을 줄 필요가 있다.

따라서 steer = k * theta

하지만 위의 이미지에서와 같이 theta를 얼마나 추종할지는 상황에 따라 다르다.

Case 1

: abs(x[1])이 abs(x[0])보다 큰 경우이다.

이 경우 앞으로 갈 수록 차선에서 멀어지므로 경로를 이탈하지 않게 하기 위해 theta에 대한 추종도를 높여야 한다.

따라서 abs(x[1]) - abs(x[0])가 양의 값을 가질 때 k는 커져야 한다.

앞으로 abs(x[1]) - abs(x[0])를 dx로 지칭한다.

Case 2

: abs(x[0])가 abs(x[1])보다 큰 경우이다.

이 경우 앞으로 갈 수록 차선에 가까워지므로 theta에 과하게 추종하지 않아도 차선에 가까워진다.

따라서 dx가 음의 값을 가질 때 k는 작아져야 한다.

Case 3 및 theta가 0에 가까울 때

: 이 경우 theta를 그대로 추종하면 된다. 따라서 wheel의 각변위를 theta와 일치시키면 되는데,

servo motor의 각변위가 그대로 wheel의 각변위가 되지 않기 때문에 이를 고려하여 실험적으로 적절한 값을 곱해서 servo motor에 명령을 주었다.

해당 코드에서는 theta에 k = 1.6~1.7의 값을 곱해서 명령을 주었을 때 wheel의 각변위가 theta에 가깝게 나왔다.

=> 따라서 dx의 값이 커질 수록 k의 값도 커지는 정비례 관계를 가지면 된다.

이를 구현하기 위해 어떤 함수를 사용해도 좋지만, 해당 코드는 dx = 0을 기준으로 k의 값이 급격하게 변하게 하기 위해 sigmoid 함수를 사용하였다.

=> steering_angle = theta * (1.5 + 0.4 * sigmoid(dx))

x에 의한 조향 theta만 추종할 시 차선이 화면의 가장자리에 걸려있어도 이를 맞추려는 노력을 하지 않는다. 따라서 차선을 계속 인식하기 위해 x에 의한 조향을 추가할 필요가 있다.

하지만 x에 의한 추종이 과할 경우 조향값이 진동하게 된다. 따라서 카메라가 차선을 놓치는 상황에 가까워질 수록 급격하게 x의 추종도를 키워야한다.

따라서 steer = k * f(x)

해당 코드는 간단하게 parabolic 함수 (2차 그래프)를 이용해 이를 구현하였다.

x_avg = (x[0] + x[1]) / 2

steering_angle = (x_avg / x_max)^2 * steering_max

(위 식은 값의 크기만 반영한 것이며 부호도 곱해주어야한다. 이는 소스코드에서 확인할 수 있다)

최종 steer 값 steering_angle = theta * (1.5 + 0.4 * sigmoid(dx)) + (x_avg / x_max)^2 * steering_max * x_sign

(x_sign = x / abs(x)로 크기는 1, 부호는 x와 같은 변수)
