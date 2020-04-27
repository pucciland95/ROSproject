#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "project/distance.h"
#include "math.h"

class server{

    public:

        server(){

            sub1.subscribe(n, "/car/EMU_pose", 1);
            sub2.subscribe(n, "/obstacle/EMU_pose", 1);

            pSync.reset(new sync(mySyncPolicy(10), sub1, sub2));
            pSync->registerCallback(boost::bind(&server::callback, this, _1, _2));

            service = n.advertiseService("distanceSrv", &server::distance, this);

        }

        void callback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2){
        
            pos1 = *msg1;
            pos2 = *msg2;
            
        }

        bool distance(project::distance::Request &req, project::distance::Response &res){

            float x1 = pos1.pose.pose.position.x;
            float x2 = pos2.pose.pose.position.x;

            float y1 = pos1.pose.pose.position.y;
            float y2 = pos2.pose.pose.position.y;

            float z1 = pos1.pose.pose.position.z;
            float z2 = pos2.pose.pose.position.z;

            res.distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2)); 

            ROS_INFO("Distance = %f", res.distance);

            return true;
        }


    private:
        ros::NodeHandle n;

        ros::ServiceServer service;
        
        message_filters::Subscriber<nav_msgs::Odometry> sub1;
        message_filters::Subscriber<nav_msgs::Odometry> sub2;

        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> mySyncPolicy;
        typedef message_filters::Synchronizer<mySyncPolicy> sync;

        boost::shared_ptr<sync> pSync;

        nav_msgs::Odometry pos1;
        nav_msgs::Odometry pos2;

};




int main(int argc, char** argv){

    ros::init(argc, argv, "distance_server");

    server server;

    ros::spin();    

}



