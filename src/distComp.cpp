#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class pub_sub_filter{

    public:
        pub_sub_filter(){

            message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "/car/EMU_pose", 1);
            message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "/obstacle/EMU_pose", 1);

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> mySyncPolicy;

            message_filters::Synchronizer<mySyncPolicy> sync(mySyncPolicy(10), sub1, sub2);
            sync.registerCallback(boost::bind(&pub_sub_filter::distanceCallback, _1, _2));
        
            pub = n.advertise<float>("distance", 1);
            pub.publish(distance);
        }



        void pub_sub_filter::distanceCallback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2){

            //ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->pose.pose.position.x ,msg1->pose.pose.position.y, msg1->pose.pose.position.z, msg2->pose.pose.position.x ,msg2->pose.pose.position.y, msg2->pose.pose.position.z);
            float x1 = msg1->pose.pose.position.x;
            float x2 = msg2->pose.pose.position.x;

            float y1 = msg1->pose.pose.position.y;
            float y2 = msg2->pose.pose.position.y;

            float z1 = msg1->pose.pose.position.z;
            float z2 = msg2->pose.pose.position.z;
            
            distance = sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2) );
        }

    private:

        ros::NodeHandle n;

        ros::Publisher pub;

        //ros::Timer timer;

        float distance;


};

int main(int argc, char** argv){

    ros::init(argc, argv, "sub_sync");

    pub_sub_filter pub_sub_filter;

    ros::spin();
    
    return 0;
}




