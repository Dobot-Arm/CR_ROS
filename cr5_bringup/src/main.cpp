/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <cr5_bringup/cr5_robot.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "CR5Robot");

    try
    {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();

        sensor_msgs::JointState joint_state_msg;
        ros::Publisher joint_state_pub = node.advertise<sensor_msgs::JointState>("/joint_states", 100);

        for (uint32_t i = 0; i < 6; i++)
        {
            joint_state_msg.position.push_back(0.0);
            joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
        }

        CR5Robot robot(private_node, "/flow_joint_trajectory/follow_joint_trajectory");

        double rate_vale = private_node.param("JointStatePublishRate", 10);

        robot.init();
        ros::Rate rate(rate_vale);
        double position[6];
        while (ros::ok())
        {
            robot.getJointState(position);
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.header.frame_id = "dummy_link";

            for (uint32_t i = 0; i < 6; i++)
                joint_state_msg.position[i] = position[i];

            joint_state_pub.publish(joint_state_msg);

            rate.sleep();
        }
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        return -1;
    }

    return 0;
}
