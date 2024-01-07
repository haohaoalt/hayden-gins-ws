// eskf_fusion_node.cpp
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <fstream>
#include "eskf_fusion/ESKFFlow.h" 
// Assuming you have ESKFFlow class in a header file named ESKFFlow.h

class ESKFFusionNode {
public:
    ESKFFusionNode() {
        // Initialize ESKFFlow with appropriate paths
        eskf_flow_ = std::make_unique<ESKFFlow>("/path/to/config_file.yaml", "/path/to/data_file.txt");

        // Subscribe to GNSS fix and IMU topics
        gnss_sub_ = nh_.subscribe("/gps/fix", 10, &ESKFFusionNode::gnssCallback, this);
        imu_sub_ = nh_.subscribe("/imu", 10, &ESKFFusionNode::imuCallback, this);
    }

    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
        // Process GNSS data and update ESKF
        // ...
        // You need to implement this part based on your ESKFFlow class
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        // Process IMU data and update ESKF
        // ...
        // You need to implement this part based on your ESKFFlow class
    }

    void run() {
        // Main loop or any other logic you need
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber imu_sub_;
    std::unique_ptr<ESKFFlow> eskf_flow_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "eskf_fusion_node");
    ESKFFusionNode node;
    node.run();
    return 0;
}

