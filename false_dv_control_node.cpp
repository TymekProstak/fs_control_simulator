#include <ros/ros.h>
#include <dv_interfaces/Control.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_false_dv_control_node");
    ros::NodeHandle nh;

    ros::Publisher control_pub = nh.advertise<dv_interfaces::Control>("/dv_board/control", 1);

    ros::Rate rate(40); // 40 Hz
    while (ros::ok()) {
        dv_interfaces::Control msg;
        msg.move_type =  false;
        msg.movement = 25.0  ;
        msg.steeringAngle_rad =     ;
        msg.service_brake =   
        msg.finished =   false;

        control_pub.publish(msg);
        ROS_INFO("Published fake control message");

        rate.sleep();
    }

    return 0;
}