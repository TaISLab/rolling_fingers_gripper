#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <iostream>

void publishFSMState(int &state, ros::Publisher &fsm_pub)
{
    // Creating MSG objects
    std_msgs::Int16 fsm_state;

    // data assignation
    fsm_state.data = state;

    // Publishing
    fsm_pub.publish(fsm_state);
}

int main(int argc, char *argv[])
{
    int state;
    int prev_state = 0;

    // ROS node init
    ros::init(argc, argv, "fsm_4_fingers_node");
    ros::NodeHandle nh;

    // Publishers and subscribers creation
    ros::Publisher fsm_state_publisher = nh.advertise<std_msgs::Int16>("fsm_state_4fingers", 1);

    // ROS freq = 1000 Hz
    ros::Rate loop_rate(1000);

    while (ros::ok())
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
                    ros::Duration(0.5).sleep(); // wait for exiting
                    break;
                }
            }

            prev_state = state;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
