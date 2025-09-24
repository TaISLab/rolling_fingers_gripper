#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <dynamixel_ros2.h>
#include <iostream>

dynamixelMotor motorJ0, motorJ1, motorJ2, motorJ10, motorJ11, motorJ12;
int fsm_state = 0;
double rotation_time;
double rotation_duration = 6;
double time_now;

void publishMotorStatus(dynamixelMotor &motor,
                        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &pos_pub,
                        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &vel_pub,
                        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &curr_pub)
{
    // Creating MSG objects
    std_msgs::msg::Float32 pos_msg;
    std_msgs::msg::Float32 vel_msg;
    std_msgs::msg::Float32 curr_msg;

    // Getting params from dmxl
    float position = static_cast<float>(motor.getPresentPosition());
    float velocity = static_cast<float>(motor.getPresentVelocity());
    float current = static_cast<float>(motor.getPresentCurrent());

    // data assignation
    pos_msg.data = position;
    vel_msg.data = velocity;
    curr_msg.data = current;

    // Publishing
    pos_pub->publish(pos_msg);
    vel_pub->publish(vel_msg);
    curr_pub->publish(curr_msg);
}

// Callback when some data was published in 'pos_user_input'
void fsmStateCallBack(const std_msgs::msg::Int16::SharedPtr msg)
{
    fsm_state = msg->data;
    if (fsm_state == 5 || fsm_state == 6)
    {
        rotation_time = rclcpp::Clock().now().seconds();
    }
}

void torqueEnabled()
{
    motorJ0.setTorqueState(true);
    motorJ1.setTorqueState(true);
    motorJ2.setTorqueState(true);
    motorJ10.setTorqueState(true);
    motorJ11.setTorqueState(true);
    motorJ12.setTorqueState(true);
}

void torqueDisabled()
{
    motorJ0.setTorqueState(false);
    motorJ1.setTorqueState(false);
    motorJ2.setTorqueState(false);
    motorJ10.setTorqueState(false);
    motorJ11.setTorqueState(false);
    motorJ12.setTorqueState(false);
}

int main(int argc, char *argv[])
{
    // Define port, rate and protocol
    // Default values
    char *port_name = const_cast<char *>("/dev/ttyUSB0");
    int baud_rate = 4500000;
    float protocol_version = 2.0f;

    // ROS2 init (do this before iniComm so library logging works)
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("rolling_3_fingers");

    // Init communication
    dynamixelMotor::iniComm(port_name, protocol_version, baud_rate);
    motorJ0 = dynamixelMotor("J0", 0);
    motorJ1 = dynamixelMotor("J1", 1);
    motorJ2 = dynamixelMotor("J2", 2);
    motorJ10 = dynamixelMotor("J10", 10);
    motorJ11 = dynamixelMotor("J11", 11);
    motorJ12 = dynamixelMotor("J12", 12);

    // Set control table
    motorJ0.setControlTable();
    motorJ1.setControlTable();
    motorJ2.setControlTable();
    motorJ10.setControlTable();
    motorJ11.setControlTable();
    motorJ12.setControlTable();

    // Define the control mode for each motor
    motorJ0.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ2.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ10.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ11.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ12.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);

    // Set joint velocity limit
    float MAX_VELOCITY = 44.0f;
    motorJ10.setVelLimit(MAX_VELOCITY);
    motorJ11.setVelLimit(MAX_VELOCITY);
    motorJ12.setVelLimit(MAX_VELOCITY);

    // Set joint position PWM limits (change the numbers for your gripper)
    motorJ0.setPWMLimit(15);
    motorJ1.setPWMLimit(20);
    motorJ2.setPWMLimit(15);

    // Enable Torque
    torqueEnabled();

    // Open and closed joint values
    float motor0_open = 240;
    float motor1_open = 43;
    float motor2_open = 327;
    float motor0_closed = 41;
    float motor1_closed = 216;
    float motor2_closed = 150;

    // Velocity values
    float slow_velocity = 20;
    [[maybe_unused]] float normal_velocity = 30;
    [[maybe_unused]] float fast_velocity = 40;

    // State 0: gripper open, not rotating
    motorJ0.setGoalPosition(motor0_open);
    motorJ1.setGoalPosition(motor1_open);
    motorJ2.setGoalPosition(motor2_open);
    motorJ10.setGoalVelocity(0);
    motorJ11.setGoalVelocity(0);
    motorJ12.setGoalVelocity(0);

    // Publishers and subscribers creation
    auto J0_pos_publisher = nh->create_publisher<std_msgs::msg::Float32>("J0_position", rclcpp::QoS(1));
    auto J0_vel_publisher = nh->create_publisher<std_msgs::msg::Float32>("J0_velocity", rclcpp::QoS(1));
    auto J0_curr_publisher = nh->create_publisher<std_msgs::msg::Float32>("J0_current", rclcpp::QoS(1));
    auto J1_pos_publisher = nh->create_publisher<std_msgs::msg::Float32>("J1_position", rclcpp::QoS(1));
    auto J1_vel_publisher = nh->create_publisher<std_msgs::msg::Float32>("J1_velocity", rclcpp::QoS(1));
    auto J1_curr_publisher = nh->create_publisher<std_msgs::msg::Float32>("J1_current", rclcpp::QoS(1));
    auto J2_pos_publisher = nh->create_publisher<std_msgs::msg::Float32>("J2_position", rclcpp::QoS(1));
    auto J2_vel_publisher = nh->create_publisher<std_msgs::msg::Float32>("J2_velocity", rclcpp::QoS(1));
    auto J2_curr_publisher = nh->create_publisher<std_msgs::msg::Float32>("J2_current", rclcpp::QoS(1));
    auto J10_pos_publisher = nh->create_publisher<std_msgs::msg::Float32>("J10_position", rclcpp::QoS(1));
    auto J10_vel_publisher = nh->create_publisher<std_msgs::msg::Float32>("J10_velocity", rclcpp::QoS(1));
    auto J10_curr_publisher = nh->create_publisher<std_msgs::msg::Float32>("J10_current", rclcpp::QoS(1));
    auto J11_pos_publisher = nh->create_publisher<std_msgs::msg::Float32>("J11_position", rclcpp::QoS(1));
    auto J11_vel_publisher = nh->create_publisher<std_msgs::msg::Float32>("J11_velocity", rclcpp::QoS(1));
    auto J11_curr_publisher = nh->create_publisher<std_msgs::msg::Float32>("J11_current", rclcpp::QoS(1));
    auto J12_pos_publisher = nh->create_publisher<std_msgs::msg::Float32>("J12_position", rclcpp::QoS(1));
    auto J12_vel_publisher = nh->create_publisher<std_msgs::msg::Float32>("J12_velocity", rclcpp::QoS(1));
    auto J12_curr_publisher = nh->create_publisher<std_msgs::msg::Float32>("J12_current", rclcpp::QoS(1));

    auto fsm_state_subscriber = nh->create_subscription<std_msgs::msg::Int16>(
        "fsm_state_3fingers", rclcpp::QoS(1), fsmStateCallBack);

    // ROS freq = 300 Hz
    rclcpp::Rate loop_rate(300);

    while (rclcpp::ok())
    {
        switch (fsm_state)
        {
        case 0:
            // State 0: Gripper open
            motorJ0.setGoalPosition(motor0_open);
            motorJ1.setGoalPosition(motor1_open);
            motorJ2.setGoalPosition(motor2_open);
            break;
        case 1:
            // State 1: gripper closed
            motorJ0.setGoalPosition(motor0_closed);
            motorJ1.setGoalPosition(motor1_closed);
            motorJ2.setGoalPosition(motor2_closed);
            break;
        case 2:
            // State 2: Rotate left
            motorJ10.setGoalVelocity(slow_velocity);
            motorJ11.setGoalVelocity(-slow_velocity);
            motorJ12.setGoalVelocity(slow_velocity);
            break;
        case 3:
            // State 3: Rotate right
            motorJ10.setGoalVelocity(-slow_velocity);
            motorJ11.setGoalVelocity(slow_velocity);
            motorJ12.setGoalVelocity(-slow_velocity);
            break;
        case 4:
            // State 4: Not rotate
            motorJ10.setGoalVelocity(0);
            motorJ11.setGoalVelocity(0);
            motorJ12.setGoalVelocity(0);
            break;
        case 5:
            // State 5: Rotate left and right
            time_now = rclcpp::Clock().now().seconds();
            if (time_now - rotation_time < rotation_duration / 2)
            {
                motorJ10.setGoalVelocity(slow_velocity);
                motorJ11.setGoalVelocity(-slow_velocity);
                motorJ12.setGoalVelocity(slow_velocity);
            }
            else if (time_now - rotation_time > rotation_duration / 2 && time_now - rotation_time < rotation_duration)
            {
                motorJ10.setGoalVelocity(-slow_velocity);
                motorJ11.setGoalVelocity(slow_velocity);
                motorJ12.setGoalVelocity(-slow_velocity);
            }
            else
            {
                motorJ10.setGoalVelocity(0);
                motorJ11.setGoalVelocity(0);
                motorJ12.setGoalVelocity(0);
                rotation_time = rclcpp::Clock().now().seconds();
            }
            break;
        case 6:
            // State 6: Ball rotation
            time_now = rclcpp::Clock().now().seconds();
            if (time_now - rotation_time < rotation_duration / 2)
            {
                motorJ10.setGoalVelocity(slow_velocity);
                motorJ11.setGoalVelocity(slow_velocity);
                motorJ12.setGoalVelocity(slow_velocity);
            }
            else if (time_now - rotation_time > rotation_duration / 2 && time_now - rotation_time < rotation_duration)
            {
                motorJ10.setGoalVelocity(-slow_velocity);
                motorJ11.setGoalVelocity(-slow_velocity);
                motorJ12.setGoalVelocity(-slow_velocity);
            }
            else
            {
                rotation_time = rclcpp::Clock().now().seconds();
            }
            break;
        }

        RCLCPP_INFO(nh->get_logger(), "FSM state is: %d", fsm_state);

        if (fsm_state == 7)
        {
            RCLCPP_INFO(nh->get_logger(), "Switching off and exiting...");
            torqueDisabled();
            break;
        }

        publishMotorStatus(motorJ0, J0_pos_publisher, J0_vel_publisher, J0_curr_publisher);
        publishMotorStatus(motorJ1, J1_pos_publisher, J1_vel_publisher, J1_curr_publisher);
        publishMotorStatus(motorJ2, J2_pos_publisher, J2_vel_publisher, J2_curr_publisher);
        publishMotorStatus(motorJ10, J10_pos_publisher, J10_vel_publisher, J10_curr_publisher);
        publishMotorStatus(motorJ11, J11_pos_publisher, J11_vel_publisher, J11_curr_publisher);
        publishMotorStatus(motorJ12, J12_pos_publisher, J12_vel_publisher, J12_curr_publisher);

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
