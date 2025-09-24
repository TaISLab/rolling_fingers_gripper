#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <iostream>
#include <thread>
#include <chrono>

void publishFSMState(int &state, rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr &fsm_pub)
{
    // Creating MSG objects
    std_msgs::msg::Int16 fsm_state;

    // data assignation
    fsm_state.data = state;

    // Publishing
    fsm_pub->publish(fsm_state);
}

int main(int argc, char *argv[])
{
    int state;
    int prev_state = 0;

    // ROS2 node init
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("fsm_3_fingers_node");

    // Publishers and subscribers creation
    auto fsm_state_publisher = nh->create_publisher<std_msgs::msg::Int16>("fsm_state_3fingers", 1);

    // ROS freq = 1000 Hz
    rclcpp::Rate loop_rate(1000);

    while (rclcpp::ok())
    {
        // Wait for user to hit a key
        printf("Select the next state:\n");
        printf("0: Open gripper\n");
        printf("1: Close gripper\n");
        printf("2: Rotate Left\n");
        printf("3: Rotate right\n");
        printf("4: No Rotation\n");
        printf("5: Rotate left and right\n");
        printf("6: Ball rotation\n");
        printf("7: Exit\n");
        std::cin >> state;
        if (state != 0 && state != 1 && state != 2 && state != 3 && state != 4 && state != 5 && state != 6 && state != 7)
        {
            printf("Wrong state selection\n");
        }
        else
        {
            printf("Going to state: %d\n", state);
            if (state != prev_state)
            {
                publishFSMState(state, fsm_state_publisher);
                if (state == 7)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for exiting
                    break;
                }
            }

            prev_state = state;
        }

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
