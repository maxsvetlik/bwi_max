/*


TODO:
Parameterize the robot distance multiplier (ie the 80% value in calculate distance)
Generalize code, make it useful for other applications
Publish information while the goal is executing
*/


#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch_human/FetchHumanAction.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "pcl_perception/PeopleDetectionSrv.h"

#include <vector>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

//for speech recognition and synthesis
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
//#include "sound_play/sound_play.h"

class FetchHumanAction{
  
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<fetch_human::FetchHumanAction> as_;
  std::string action_name_;
  std_msgs::String voiceCmd;
  //messages for feeback
  fetch_human::FetchHumanFeedback feedback_;
  fetch_human::FetchHumanResult result_;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac_;
  //variables to hold current position on map
  geometry_msgs::PoseStamped curPosition;
public:

  FetchHumanAction(std::string name) : 
  as_(nh_, name, boost::bind(&FetchHumanAction::executeCB, this, _1), false), 
  action_name_(name), ac_("move_base", true)
  {
  	ROS_INFO("Action server started.");
  	geometry_msgs::PoseStamped myPose = getCurrentPose();
  	ROS_INFO("My current pose: x = %f, y = %f", myPose.pose.position.x, myPose.pose.position.y);
  	as_.start();
  }
  ~FetchHumanAction(void)
  {

  }
  
  void executeCB(const fetch_human::FetchHumanGoalConstPtr &goal){
  	ROS_INFO("Action service callback started...");
  	
  	//helper vars
  	bool success = true;
  	bool done = false;
  	bool found = false;
  	double turnMax = .25 * M_PI;
  	double turnedSoFar = 0.0;

  	//for person_perception service call
  	std::string service_name = "segbot_pcl_person_detector/people_detection_service";	
  	std::string command = "detect_people";       //<< Text to change in accordance to Jivko's service
  	ros::ServiceClient serviceClient = nh_.serviceClient<pcl_perception::PeopleDetectionSrv>(service_name, true);
 	//checks if service is available
    if (!serviceClient) {
        ROS_FATAL("Persistent service connection to %s failed", serviceClient.getService().c_str());
    }
    
    ROS_INFO("Starting to turn...");
		
  	std::vector<geometry_msgs::PoseStamped> pa;
  	pcl_perception::PeopleDetectionSrv srv;
	//will rotate around 360 degrees looking for humans
	while(!found && (2*M_PI) >= turnedSoFar){
	  ROS_INFO("...next turn");
	  srv.request.command = command;
      pa = callPlanningService(serviceClient, srv);
	    
	  if(!pa.empty()){
	    found = true;
	    ROS_INFO("I've found %i many poses", (int)(pa.size()));  
	  	}
      if(!found){
  	    turnedSoFar += turnMax;
  	    rotate(turnMax);
	  }
	}

	ROS_INFO("Done turning...");
	
	//seek human if you've found any
  	if(found){
  	  //convert pose array from kinect frame to /map frame
  	  //geometry_msgs::PoseArray mapPoses = convertFrames(pa);
 	  geometry_msgs::PoseStamped calcPose = findClosestPose(pa);
 	  geometry_msgs::PoseStamped target = reduceDistance(calcPose);
 	  seek(target);
	  done = true;
	}
  	else{
  		ROS_INFO("No humans found after a full rotation. Exiting...");
  		done = true;
  		success = false;
  	}
    if(success){
      as_.setSucceeded(result_);
    }
    else
      as_.setAborted(result_);
  }

/*	//callback
  void speech(const std_msgs::String& voiceMsg){
	voiceCmd = voiceMsg;
  }

//TODO
  //starts the voice module, begins subscribing to the output.
  void startListen(){
  	ros::NodeHandle n;
  	//FetchHumanAction listener;
  	ros::Subscriber speech_sub_= n.subscribe("/recognizer/output", 1, boost::bind(&FetchHumanAction::speech, this, _1));

    sound_play::SoundClient soundMod;
    soundMod.say("Beep beep, I am now listening to you.");
    ros::Duration(3.0).sleep();
    soundMod.say("Are you a human?");

  	ros::Duration dur(5); //
 	ros::Time start = ros::Time::now();
  	bool answered = false;
  	while(ros::Time::now() - start < dur && ros::ok() && !answered){
  		if(::voiceCmd.data == "yes"){
  			answered = true;
  			soundMod.say("Oh good, I'm glad you weren't an error!");
		}
  		else if(::voiceCmd.data == "no"){
  			answered = true;
  			soundMod.say("Hmm, are you sure about that?");
  		}
  	}
  }*/
  /*
  //this function sends goals, or does other activities based on what it hears from
  //speech subscriber
  void recognition(const std_msgs::String& voice){
  	::fetch_human = speak;
  	ros::NodeHandle n;
  }*/

  geometry_msgs::PoseStamped getCurrentPose(){
	  ros::NodeHandle nh;
	  tf::TransformListener listener;
	  tf::StampedTransform transform;
	  geometry_msgs::PoseStamped tempPose;
	  try{
		  listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(2.0)); 
		  listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
		  ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
		  tempPose.pose.position.x = transform.getOrigin().x();
		  tempPose.pose.position.y = transform.getOrigin().y();
		  tempPose.pose.position.z = 0;
		  tempPose.pose.orientation.w = transform.getRotation().getW();
		  
		  tf::Vector3 angle = transform.getRotation().getAxis();
		  tempPose.pose.orientation.x = angle[0];
		  tempPose.pose.orientation.y = angle[1];
		  tempPose.pose.orientation.z = angle[2]; 
	  }
	  catch (tf::TransformException ex){
	        ROS_ERROR("Error: %s", ex.what());
	  }
	  
	  return tempPose;
  }
  
  //Prints input pose. Useful for debugging
  void print_pose(geometry_msgs::Pose P){
	  ROS_INFO("pose: p(%f, %f, %f), orientation(%f, %f, %f, %f)",
				P.position.x,
				P.position.y,
				P.position.z,
				P.orientation.x,
				P.orientation.y,
				P.orientation.z,
				P.orientation.w);  
  }


  //Method to take in a PoseStamped message. 
  //Returns a poseStamped message with its distance reduced by TODO THE PARAMETER
  //based on its distance from where the robot currently is.
  //Assumes that curPosition has been set in a previous method (namely, pickHumanPose)
  geometry_msgs::PoseStamped reduceDistance(geometry_msgs::PoseStamped &destination){
  	  double curDistance, destX, destY, curX, curY = 0.0;
	  double smallestDistance, smallestX, smallestY = 99999; //arbitary large value
	  //curPosition = getCurrentPose();
	  geometry_msgs::PoseStamped resultPose;
	  curX = curPosition.pose.position.x;
	  curY = curPosition.pose.position.y;

	  destX = destination.pose.position.x;
	  destY = destination.pose.position.y;


	    //Found smallest pose. Using distances, use multiplier to reduce x and y values. 
	    //TODO parameterize the multiplier
	  resultPose.pose.position.x = curX + ((destX-curX) * .7);
	  resultPose.pose.position.y = curY + ((destY-curY) * .7);
		
	  resultPose.pose.position.z =0.0; //flatten it to the ground
	  resultPose.pose.orientation = destination.pose.orientation;
	  return resultPose;
  }


  	  //Method takes in an array of poses and returns the closest pose.
  geometry_msgs::PoseStamped findClosestPose(std::vector<geometry_msgs::PoseStamped> &mapPoses){
	  geometry_msgs::PoseStamped smallestPose;
	  double curDistance, initX, initY, curX, curY = 0.0;
	  double smallestX, smallestY = 0.0;
	  double smallestDistance = 10000.0;  //arbitary large value
	  geometry_msgs::PoseStamped f;
	  
	  curPosition = getCurrentPose();
	  initX = curPosition.pose.position.x;
	  initY = curPosition.pose.position.y;
		
	  
	  for(int i = 0; i < mapPoses.size(); i++){
		  f = mapPoses.at(i);
		  print_pose(f.pose); 
		  curX = f.pose.position.x;
		  curY = f.pose.position.y;
		  curDistance = sqrt((pow(initX - curX, 2) + pow(initY - curY, 2))); //distance formula
		  
		  ROS_INFO("%f, %f, %f, %f, \t %f",initX,initY,curX,curY,curDistance);
		  }
		  if(curDistance < smallestDistance){ //find smallest distance
		  	smallestDistance = curDistance;
		  	smallestX = curX;
		  	smallestY = curY;
		  	smallestPose = f;
		  }
	  
	  ROS_INFO("Chose closest pose:");
	  print_pose(smallestPose.pose);
	  
	  return smallestPose;
   }


	//makes the call to pcl_perception service
  std::vector<geometry_msgs::PoseStamped> callPlanningService(ros::ServiceClient &serviceClient, pcl_perception::PeopleDetectionSrv srv)
	{
		std::vector<geometry_msgs::PoseStamped> resultPoses;
	    
	    // Perform the actual call
	    if (serviceClient.call(srv)) {
	        if (!srv.response.poses.empty()) {
	            forEach(const geometry_msgs::PoseStamped &p, srv.response.poses) {
	                ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
	                resultPoses.push_back(p);
	            }
	        }
	        else {
	            ROS_INFO("No Humans found");
	        }
	        return resultPoses;
	    }
	    else
	    	ROS_WARN("Service call failed!");
	    return resultPoses;
	}


	//Method to convert poses from /kinect_rgb_optical_frame to /map frame via tf listener
	//returns poseArray of /map poses
	//Not used at current time.
	geometry_msgs::PoseArray convertFrames(geometry_msgs::PoseArray pa){
	  geometry_msgs::PoseArray tempArray;
	  geometry_msgs::Pose tempPose;
	  ros::NodeHandle nh;
	  tf::TransformListener listener;
	  tf::StampedTransform transform;

	  forEach(geometry_msgs::Pose &f, pa.poses){
		  try
		    {
				ROS_INFO("Attempting to convert frames...");
				//create a stampedPose, for the /kinect_rgb_optical frame.
				//fill out header for the tf transformation
		    	geometry_msgs::PoseStamped stampIn;
		    	stampIn.header.seq = pa.header.seq;
		    	stampIn.header.frame_id = "/nav_kinect_rgb_optical_frame";
		    	stampIn.header.stamp = ros::Time::now();
		    	stampIn.pose = f;
		    	stampIn.pose.orientation = tf::createQuaternionMsgFromYaw(0);
				
				//stampedPose result for the tf transform
		    	geometry_msgs::PoseStamped stampOut;
				
				//waits for a transformation to be avilaible 
		    	listener.waitForTransform("nav_kinect_rgb_optical_frame", "map", ros::Time::now(), ros::Duration(3.0)); 
		    	// ^ note about above. some early frames will be lost since tf only keeps about 10 seconds of data.
		    	//ie, looking at 15 frames may be too many
		    	
		        //transforms the frames
		        listener.transformPose("map", stampIn, stampOut);

		        //add transformed poses to pose array
				tempArray.poses.push_back(stampOut.pose);
		        ROS_INFO("Successfully transformed a pose!");
		    }
		  catch (tf::TransformException ex)
		    {
		        ROS_ERROR("Error: %s", ex.what());
		    }
		}
	  return tempArray;
	}


	//rotates the robot once qon the given value
	bool rotate(double yawValue){
	      move_base_msgs::MoveBaseGoal goal;
	      goal.target_pose.header.stamp = ros::Time::now();
	      goal.target_pose.header.frame_id = "/base_link";

	      goal.target_pose.pose.position.x = 0.0;
	      goal.target_pose.pose.position.y = 0.0;
	      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yawValue);

	      ROS_INFO("Rotating...");
	      ac_.sendGoal(goal);
	      ac_.waitForResult();
	    return true;
	}


	//Sends goal to move_base. Moves the robot to the coordinates of the input PoseStamped
	bool seek(geometry_msgs::PoseStamped &tp){
	    move_base_msgs::MoveBaseGoal goal;
	    goal.target_pose.header.stamp = ros::Time::now();
	    goal.target_pose.header.frame_id = "map";
	    goal.target_pose.pose.position.x = tp.pose.position.x;
	    goal.target_pose.pose.position.y = tp.pose.position.y;
	    
	    //get my current pose and preserve orientation
		//geometry_msgs::PoseStamped curPose = getCurrentPose();
		goal.target_pose.pose.orientation = curPosition.pose.orientation;

		ROS_INFO("Current pose:");
		print_pose(curPosition.pose);
	    ROS_INFO("Attempting to reach human. Goal pose:");
	    print_pose(goal.target_pose.pose);
	    ac_.sendGoal(goal);
	    ac_.waitForResult();

	    if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	      ROS_INFO("Succesfully reached human.");
	      return true;
	    }
	    else
	      ROS_INFO("Failed to reach human.");
	    return false;
	}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "human_seeker");
  ros::NodeHandle n;
  FetchHumanAction human_seeker(ros::this_node::getName());
  ros::spin();
  return 0;
}
