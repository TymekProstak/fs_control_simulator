#include <ros/ros.h>
#include <dv_interfaces/Control.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_false_dv_control_node");
    ros::NodeHandle nh;

    ros::Publisher control_pub = nh.advertise<dv_interfaces::Control>("/dv_board/control", 1);

    ros::Rate rate(40); // 40 Hz
    double amplitude = 2.0 * M_PI / 180.0; // Amplituda ±5 stopni w radianach
    double frequency = 0.5; // Częstotliwość oscylacji 

    while (ros::ok()) {
        dv_interfaces::Control msg;
        msg.move_type =  static_cast<bool>(false);
        msg.movement = static_cast<float> (20.0)  ; // 10 procent momentu
        msg.steeringAngle_rad = amplitude * std::sin(2 * M_PI * frequency * ros::Time::now().toSec());
    
        msg.serviceBrake =  static_cast<uint8_t>( false); 
        msg.finished =   static_cast<bool>(false);

        control_pub.publish(msg);
        //ROS_INFO("Published fake control message"); to bez

        rate.sleep();
    }

    return 0;
}