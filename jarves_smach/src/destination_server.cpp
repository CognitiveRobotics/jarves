//ros specific
#include <ros/ros.h>
#include "ros/console.h"

//jarves specific
#include <jarves/Destination.h>

//c++specific
#include <string>

bool lookup_callback(jarves::Destination::Request &req, jarves::Destination::Response &res){

    std::string destination_string = req.destination;
    ROS_INFO("Received destination string = %s\n", destination_string.c_str());
    res.success = true;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "destination_lookup_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("destination_lookup", lookup_callback);

    ros::spin();
    return 0;
}
