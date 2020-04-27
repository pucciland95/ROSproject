#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "project/distance.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "client");

    //client client;

    ros::NodeHandle n;
    ros::ServiceClient clientObj = n.serviceClient<project::distance>("distanceSrv");
    project::distance srv;

    if (clientObj.call(srv)){

        ROS_INFO("Actual distance %f", srv.response.distance);

    }

}
