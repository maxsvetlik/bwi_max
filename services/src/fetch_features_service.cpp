#include <ros/ros.h>
#include <signal.h> 
#include <services/Observations.h>
#include <services/FetchFeatures.h>

bool g_caught_sigint=false;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

bool service_cb(services::FetchFeatures::Request &req, services::FetchFeatures::Response &res){
	
	
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fetch_feature_node");
    ros::NodeHandle n;
    ros::ServiceServer srv = n.advertiseService("fetch_feature_service", service_cb);

	ros::Rate r(5);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
    return 0;
}
