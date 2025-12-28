#include <ros/ros.h>
#include <dv_interfaces/Control.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_false_dv_control_node");
    ros::NodeHandle nh;

    ros::Publisher control_pub = nh.advertise<dv_interfaces::Control>("/dv_board/control", 1);

    ros::Rate rate(40); // 40 Hz
    double amplitude = 2.0 * M_PI / 180.0; // Amplituda ±5 stopni w radianach
    double frequency = 10; // Częstotliwość oscylacji 

    while (ros::ok()) {
        dv_interfaces::Control msg;
        msg.move_type = dv_interfaces::Control::TORQUE_PERCENTAGE;

        // per-wheel torque command in [%]
        const float tq_pct = 3.0f;
        msg.torque_fl = tq_pct;
        msg.torque_fr = tq_pct;
        msg.torque_rl = tq_pct;
        msg.torque_rr = tq_pct;

        msg.movement = 0.0f; // ignored in torque mode
        msg.steeringAngle_rad = - amplitude * std::sin(2 * M_PI * frequency * ros::Time::now().toSec());

        msg.serviceBrake = static_cast<uint8_t>(false);
        msg.finished = static_cast<bool>(false);

        control_pub.publish(msg);
        rate.sleep();
    }

    return 0;
}