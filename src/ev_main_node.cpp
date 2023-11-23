#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "ev_main/MotorMsg.h"
#include "ev_main/ServoMsg.h"
#include "ev_main/PingMsg.h"

using namespace cv;
using namespace std;
using namespace Eigen;

int cam_h = 320;
int cam_w = 240;

Mat reg_of_int(Mat img)
{
    int img_h = img.rows;
    int img_w = img.cols;
    double padding = 0.05;

    Point points[2][4];
    points[0][0] = Point((int)(padding * img_w), (int)(padding * img_h));
    points[0][1] = Point((int)(padding * img_w), (int)((1 - padding) * img_h));
    points[0][2] = Point((int)((1 - padding) * img_w), (int)((1 - padding) * img_h));
    points[0][3] = Point((int)((1 - padding) * img_w), (int)(padding * img_h));
    points[1][0] = Point(0, img_h);
    points[1][1] = Point(0, 0);
    points[1][2] = Point(img_w, 0);
    points[1][3] = Point(img_w, img_h);
    const Point *ppt_exclude[1] = {points[0]};
    const Point *ppt_include[1] = {points[1]};
    int npt[] = {4};
    Mat mask = Mat::zeros(img.size(), img.type());
    fillPoly(mask, ppt_include, npt, 1, Scalar(255, 255, 255), LINE_8);
    fillPoly(mask, ppt_exclude, npt, 1, Scalar(0, 0, 0), LINE_8);

    // Apply Mask to the Image
    Mat masked_img;
    bitwise_and(img, mask, masked_img);

    return masked_img;
}

Point get_disp(const Mat &masked, int &x, double &theta)
{
    std::vector<std::vector<Point>> contours;
    findContours(masked, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int potent_yellow_number = 0;
    Point potents[2];
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > 100)
        { // filter out small contours to remove noise
            Rect bounding_rect = boundingRect(contours[i]);
            Point center(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y + bounding_rect.height / 2);
            if (potent_yellow_number < 2)
                potents[potent_yellow_number++] = center;
        }
    }
    if (potent_yellow_number != 2)
        return Point(0, 0);
    if (potents[0].y == potents[1].y)
        return Point(0, 0);
    int holder = potents[0].y > potents[1].y ? 0 : 1;
    int follower = !holder;
    x = potents[holder].x - cam_w / 2;
    theta = atan2(potents[follower].x - potents[holder].x, potents[follower].y - potents[holder].y);
}

// Tunable parameters
const double Lf = 2.67; // Distance between the front of the car and its center of gravity
const double Ts = 0.1;  // Sample time
const double N = 10;    // Prediction horizon
// Linear MPC controller
void mpc_controller(double x, double theta, int &steering_angle, int &speed)
{
    // Define the system matrices
    Matrix<double, 4, 4> A;
    A << 1, Ts, 0, 0,
        0, 1, -Ts * x / pow(1 - pow(theta, 2), 0.5), 0,
        0, 0, 1, Ts,
        0, 0, -theta / pow(1 - pow(theta, 2), 0.5), 1;

    Matrix<double, 4, 2> B;
    B << 0, 0,
        Ts / m, 0,
        0, 0,
        0, Ts * v / Lf;

    Matrix<double, 4, 1> x0;
    x0 << x, 0, 0, theta;

    Matrix<double, 2, 1> u0;
    u0 << 0, v;

    // Define the cost function
    MatrixXd Q = MatrixXd::Identity(4, 4);
    MatrixXd R = MatrixXd::Identity(2, 2);

    // Predict the system over the horizon
    vector<double> x_pred(N + 1);
    vector<double> y_pred(N + 1);
    x_pred[0] = x0(0);
    y_pred[0] = x0(1);
    for (int i = 1; i <= N; i++)
    {
        Matrix<double, 4, 1> x_next = A * x0 + B * u0;
        x_pred[i] = x_next(0);
        y_pred[i] = x_next(1);
        x0 = x_next;
    }

    // Calculate the control input
    MatrixXd x_ref = MatrixXd::Ones(N + 1, 1) * x_pred[N];
    MatrixXd y_ref = Map<MatrixXd>(&y_pred[0], N + 1, 1);
    MatrixXd ref = Map<MatrixXd>(&x_pred[0], N + 1, 1);
    MatrixXd state_ref(2 * (N + 1), 1);
    state_ref << ref, y_ref;
    MatrixXd u = MatrixXd::Zero(2, 1);

    for (int i = 0; i < N; i++)
    {
        Matrix<double, 4, 1> x_err = A * x0 - state_ref.block<4, 1>(4 * i, 0);
        Matrix<double, 2, 1> u_err = u0 - u;
        u += (A.transpose() * Q * x_err + R * u_err);
    }

    // Update the control outputs
    steering_angle = atan2(Lftheta, 1 - Lftheta * x0(2));
    speed = max(min(u(1, 0), 255), -255);
    // Scale the steering angle to the range [-30, 30]
    steering_angle = max(min(steering_angle * 180 / M_PI, 30), -30);
}

uint16_t pingValue[2] = [ 9999, 9999 ];
void pingCb(const ev_main::PingMsg::ConstPtr &pingMsg)
{
    char dataStr[9] = pingMsg->data;
    pingVlaue[0] = 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ev_main_node");
    ros::NodeHandle nh;

    ros::Publisher motor_pub = nh.advertise<ev_main::MotorMsg>("motor", 100);
    ros::Publisher servo_pub = nh.advertise<ev_main::ServoMsg>("servo", 100);
    ros::Subscriber ping_sub = nh.subscribe("ping", 100, pingCb);

    ros::Rate loop_rate(100);

    ev_main::MotorMsg motorMsg;
    ev_main::ServoMsg servoMsg;
    ev_main::PingMsg pingMsg;

    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, cam_w);
    cap.set(CAP_PROP_FRAME_HEIGHT, cam_h);

    Scalar lower_yellow(20, 51, 85);
    Scalar upper_yellow(70, 255, 255);

    int x = 0, steering_angle = 0, speed = 0;
    double theta = 0.0;
    while (ros::ok())
    {
        Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        Mat roi;
        roi = reg_of_int(frame);

        Mat hsv;
        cvtColor(roi, hsv, COLOR_BGR2HSV);
        Mat yellow_mask;
        inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

        get_disp(yellow_mask, x, theta);
        mpc_controller(x, theta, steering_angle, speed);

        servoMsg.data = ("%c%3d", steering_angle > 0 ? 'l' : 'r', abs(steering_angle));
        motorMsg.data = ("%c%3d", speed > 0 ? 'f' : 'b', abs(speed));
        ROS_INFO("input %d", msg.data);
        servo_pub.publish(servoMsg);
        motor_pub.publish(motorMsg);

        loop_rate.sleep();
    }

    cap.release();
    return 0;
}

// source ~/catkin_ws/devel/setup.bash