#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>  
#include <tf/transform_broadcaster.h>
#include <limits>

class Sub{

public:
  Sub(){

    n.getParam("x0", x0);
    n.getParam("y0", y0);
    n.getParam("z0", z0);
    n.getParam("child_frame_id", childFrameId);

    xEastPrev = 0;
    yNorthPrev = 0;
    zUpPrev = 0;

    sub = n.subscribe("/swiftnav/front/gps_pose", 1000, &Sub::chatterCallback, this);
    pub = n.advertise<nav_msgs::Odometry>("EMU_pose", 1);
    timer = n.createTimer(ros::Duration(0.1), &Sub::pubCallback, this);
  }


  void pubCallback(const ros::TimerEvent&){

    // Here we need to fill the odometry msg and then send it
    // The odometry msg is composed as follows: 

    //    Header header
    //    string child_frame_id
    //    geometry_msgs/PoseWithCovariance pose
    //    geometry_msgs/TwistWithCovariance twist

    odometryMsg.header.frame_id = "world";
    odometryMsg.header.stamp = ros::Time::now();

    odometryMsg.child_frame_id = childFrameId; 

    //odometryMsg.pose.covariance = ; non settabile
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    //odometryMsg.pose.pose.orientation ; non settabile

    odometryMsg.pose.pose.position.x = xEast / 10;
    odometryMsg.pose.pose.position.y = yNorth / 10;
    odometryMsg.pose.pose.position.z = zUp / 10;

    pub.publish(odometryMsg);
  }

  void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    //ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude, msg->altitude);

    // fixed values

    double a = 6378137;
    double b = 6356752.3142;
    double f = (a - b) / a;
    double e_sq = f * (2-f);
    float deg_to_rad = 0.0174533;
    
    // input data from msg
    float latitude = msg->latitude;
    float longitude = msg->longitude;
    float h = msg->altitude;

    // fixed position (starting point that later will be converted to parameters TODO)
    //float latitude_init = 45.6311926152;
    //float longitude_init = 9.2947495255;
    //float h0 = 231.506675163;


    //lla to ecef
    float lamb = deg_to_rad*(latitude);
    float phi = deg_to_rad*(longitude);
    float s = sin(lamb);
    float N = a / sqrt(1 - e_sq * s * s);

    float sin_lambda = sin(lamb);
    float cos_lambda = cos(lamb);
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    float x = (h + N) * cos_lambda * cos_phi;
    float y = (h + N) * cos_lambda * sin_phi;
    float z = (h + (1 - e_sq) * N) * sin_lambda;
    
    // ROS_INFO("ECEF position: [%.9f,%.9f, %.9f]", x, y, z);
    // ECEF iniziali = 4410124.000000, 720727.875000, 4536098.000000

    float xd = x - x0;
    float yd = y - y0;
    float zd = z - z0;

    xEast = -sin_phi * xd + cos_phi * yd;
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    //ROS_INFO("ENU position: [%f, %f, %f]", xEast, yNorth, zUp);

    if (abs(xEast - xEastPrev) > 10 || abs(yNorth - yNorthPrev) > 10 || abs(zUp - zUpPrev) > 10){

      xEast = std::numeric_limits<double>::quiet_NaN();
      yNorth = std::numeric_limits<double>::quiet_NaN();
      zUp = std::numeric_limits<double>::quiet_NaN();

      std::cout << childFrameId + " has lost signal" << std::endl;
      std::cout << "xEast = " << xEast << " ";
      std::cout << "yNorth = " << yNorth << " ";
      std::cout << "zUp = " << zUp << std::endl;

    }
    
    xEastPrev = xEast;
    yNorthPrev = yNorth;
    zUpPrev = zUp;

    
  }

private:

  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Timer timer;

  // Initial position
  float x0;
  float y0;
  float z0;

  //Child frame id
  std::string childFrameId;

  // ENU cordinates
  float  xEast; 
  float  yNorth;
  float  zUp; 

  float xEastPrev;
  float yNorthPrev;
  float zUpPrev;

  nav_msgs::Odometry odometryMsg;

};



/*
void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude, msg->altitude);

  // fixed values

  double a = 6378137;
  double b = 6356752.3142;
  double f = (a - b) / a;
  double e_sq = f * (2-f);
  float deg_to_rad = 0.0174533;
  
  // input data from msg
  float latitude = msg->latitude;
  float longitude = msg->longitude;
  float h = msg->altitude;

  // fixed position (starting point that later will be converted to parameters TODO)
  //float latitude_init = 45.6311926152;
  //float longitude_init = 9.2947495255;
  //float h0 = 231.506675163;


  //lla to ecef
  float lamb = deg_to_rad*(latitude);
  float phi = deg_to_rad*(longitude);
  float s = sin(lamb);
  float N = a / sqrt(1 - e_sq * s * s);

  float sin_lambda = sin(lamb);
  float  cos_lambda = cos(lamb);
  float  sin_phi = sin(phi);
  float  cos_phi = cos(phi);

  float  x = (h + N) * cos_lambda * cos_phi;
  float  y = (h + N) * cos_lambda * sin_phi;
  float  z = (h + (1 - e_sq) * N) * sin_lambda;
  
  ROS_INFO("ECEF position: [%f,%f, %f]", x, y,z);
  

  // ecef to enu
 
  lamb = deg_to_rad*(latitude_init);
  phi = deg_to_rad*(longitude_init);
  s = sin(lamb);
  N = a / sqrt(1 - e_sq * s * s);

  sin_lambda = sin(lamb);
  cos_lambda = cos(lamb);
  sin_phi = sin(phi);
  cos_phi = cos(phi);

  float  x0 = (h0 + N) * cos_lambda * cos_phi;
  float  y0 = (h0 + N) * cos_lambda * sin_phi;
  float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

  float xd = x - x0;
  float  yd = y - y0;
  float  zd = z - z0;

  float  xEast = -sin_phi * xd + cos_phi * yd;
  float  yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
  float  zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

  ROS_INFO("ENU position: [%f, %f, %f]", xEast, yNorth, zUp);
  
  
}*/

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

	//ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("/swiftnav/front/gps_pose", 1000, chatterCallback);
  Sub sub;
  ros::spin();

  return 0;
}


