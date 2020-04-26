#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "project/customMsg.h"

class pub_sub_filter{

    public:
        pub_sub_filter(){

            sub1.subscribe(n, "/car/EMU_pose", 1);
            sub2.subscribe(n, "/obstacle/EMU_pose", 1);

            //typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> mySyncPolicy;

            //message_filters::Synchronizer<mySyncPolicy> sync(mySyncPolicy(10), sub1, sub2);
            pSync.reset(new sync(mySyncPolicy(10), sub1, sub2));
            pSync->registerCallback(boost::bind(&pub_sub_filter::distanceCallbackComp, this, _1, _2));
            //sync.registerCallback(boost::bind(&pub_sub_filter::distanceCallbackComp, this, _1, _2));

            pub = n.advertise<project::customMsg>("distance", 1);
            
        }

        void distanceCallbackComp(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2){

            //ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->pose.pose.position.x ,msg1->pose.pose.position.y, msg1->pose.pose.position.z, msg2->pose.pose.position.x ,msg2->pose.pose.position.y, msg2->pose.pose.position.z);
            float x1 = msg1->pose.pose.position.x;
            //ROS_INFO("This is the value of xCar: %f \n", x1);
            float x2 = msg2->pose.pose.position.x;
            //ROS_INFO("This is the value of xObstacle: %f \n", x2);

            float y1 = msg1->pose.pose.position.y;
            //ROS_INFO("This is the value of yCar: %f \n", y1);
            float y2 = msg2->pose.pose.position.y;
            //ROS_INFO("This is the value of yObstacle: %f \n", y2);


            float z1 = msg1->pose.pose.position.z;
            //ROS_INFO("This is the value of ZCar: %f \n", z1);
            float z2 = msg2->pose.pose.position.z;
            //ROS_INFO("This is the value of ZObstale: %f \n", z2);

            if(isnan(x1) || isnan(y1) || isnan(z1) || 
               isnan(x2) || isnan(y2) || isnan(z2)){
                distance = std::numeric_limits<double>::quiet_NaN();
            }

            else
                distance = sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2) );

            if (distance > 100)
                distance = std::numeric_limits<double>::quiet_NaN();
            
            //ROS_INFO("The distance is: %f \n", distance);

            project::customMsg customMsg;

            customMsg.distance = distance;

            if (distance > 5){
                customMsg.status = "Safe";
            }
            else if ( distance <= 5 && distance >= 1){
                customMsg.status = "Unsafe";
            }
            else if (distance < 1){
                customMsg.status = "Crash";
            }
            else{
                customMsg.status = "Unknown";
            }


            pub.publish(customMsg);



        }

    private:

        ros::NodeHandle n;

        message_filters::Subscriber<nav_msgs::Odometry> sub1;
        message_filters::Subscriber<nav_msgs::Odometry> sub2;

        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> mySyncPolicy;
        typedef message_filters::Synchronizer<mySyncPolicy> sync;

        boost::shared_ptr<sync> pSync;

        ros::Publisher pub;
        ros::Timer timer;

        float distance;


};

int main(int argc, char** argv){

    ros::init(argc, argv, "sub_sync");

    pub_sub_filter pub_sub_filter;

    ros::spin();
    
    return 0;
}




