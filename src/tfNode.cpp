#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

class Tf{

public:

    Tf(){
        n.getParam("child_frame_id", childFrame);
        sub = n.subscribe("/" + childFrame + "/EMU_pose", 1000, &Tf::callback, this);

    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        tf::Transform transformation;

        transformation.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transformation.setRotation(q);

        br.sendTransform(tf::StampedTransform(transformation, ros::Time::now(), "world", childFrame));
    
    }

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    tf::TransformBroadcaster br;

    std::string childFrame;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "listener");
    Tf tfNode;
    ros::spin();
    return 0;

}






