#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>


const std::string pointFileLocation = "/home/maxwell/sandbox_ws/src/random_crawler/points/points.txt";


class Crawler{
  
protected:
ros::NodeHandle nh_;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient ac_;
//variables to hold current position on map
std::vector<geometry_msgs::Pose> targetPoints;

//const int NUMBER_OF_STOPS = 1;

public:
	geometry_msgs::Pose curPosition;
	//Reads from a file and sets a vector of poses based on the file's data in the format x,y,z.
	Crawler(std::string name) : ac_("move_base", true){
		targetPoints = getPointsFromFile();
	}

	//reads points from file
	std::vector<geometry_msgs::Pose> getPointsFromFile(){
		//create poseArray here
		std::vector<geometry_msgs::Pose> poseVector;
		geometry_msgs::Pose curPose;

		//read from file here 
		std::ifstream file;
		ROS_INFO("Trying to open file...");
		double curValue;
		int counter = 0;
		file.open("/home/maxwell/catkin_ws/src/random_crawler/points/points.txt"); //make this global constant; for convience if it ever changes

		if(file.is_open()){
			ROS_INFO("File opened.");
			while(file >> curValue){
				counter++;
				if(counter == 1) //x value
					curPose.position.x = curValue;
				else if (counter == 2) //y value
					curPose.position.y = curValue;
				else if (counter == 3){ //z value
					curPose.position.z = curValue;
					counter = 0;
					curPose.orientation = tf::createQuaternionMsgFromYaw(0); //fills orientation with a temp random value. should eventually be replaced with better orientation value
					poseVector.push_back(curPose);
				}
				else
					ROS_ERROR("There was an error reading the input pose file.");
			}
			file.close();
			ROS_INFO("Added all %d poses succesfully.", (int)poseVector.size());
		}
		return poseVector;
	}

	//grabs current pose of robot and returns it in a stamped pose.
	//Note however that header information is not filled in
	//TODO: change to Pose, since header information is neglected
	void getCurrentPose(){
		ros::NodeHandle nh;
		tf::TransformListener listener;
		tf::StampedTransform transform;
		geometry_msgs::Pose tempPose;
		try{
			listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(5.0)); 
			listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
			ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
			tempPose.position.x = transform.getOrigin().x();
			tempPose.position.y = transform.getOrigin().y();
			tempPose.position.z = 0;
			tempPose.orientation.w = transform.getRotation().getW();
		  
			tf::Vector3 angle = transform.getRotation().getAxis();
			tempPose.orientation.x = angle[0];
			tempPose.orientation.z = angle[2]; 
	  }
	  catch (tf::TransformException ex){
	        ROS_ERROR("Error: %s", ex.what());
	  }
	  
	  curPosition = tempPose;
	}

	//rotates the robot once on the given value
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
	//Method takes in vector and index of the point to be arranged to the first position of the vector.
	//The points between 0 and index will be added to the end of the vector in reverse order
	//Limitation: Goes by distance "the way the crow flies" and not distance traveled (say by odometry)
	// ie:    a b c d, 2  -->   c d b a 
	void rearrangePoints(int index){
		std::vector<geometry_msgs::Pose> result;
		std::vector<geometry_msgs::Pose> temp;
		std::vector<geometry_msgs::Pose> holder = targetPoints;

		ROS_INFO("!%d", (int)holder.size());
		//holds values that will be added to the back
		for(int i = 0; i < index; i++){
			ROS_INFO("Accessing holder at %d with size %d", i, (int)holder.size());
			temp.push_back(holder[i]);
		}
		//adds points to result, starting with the point at [index]
		for(int j = index; j < (int)holder.size(); j++){
			ROS_INFO("Accessing holder at %d with size %d", j, (int)holder.size());
			result.push_back(holder[j]);
		}
		//adds the queued points to the back, in reverse order
		for(int t = 0; t < (int)temp.size(); t++){
			ROS_INFO("Accessing temp at %d with size %d", t, (int)temp.size());
			result.push_back(temp[t]);
		}
		ROS_INFO("Setting rearranged points to the global.");
		targetPoints = result;
	}


	//Method takes in an array of poses and returns the index of the closest point to your current location
	int findClosestPose(){
		geometry_msgs::PoseStamped smallestPose;
		double curDistance, initX, initY, curX, curY = 0.0;
		double smallestX, smallestY = 0.0;
		double smallestDistance = 10000.0;  //arbitary large value
		int index = 0;
		geometry_msgs::Pose f;
	  	ros::Duration(5.0).sleep();
		getCurrentPose();
		initX = curPosition.position.x;
		initY = curPosition.position.y;
	  
		for(int i = 0; i < targetPoints.size(); i++){
			f = targetPoints.at(i);
			curX = f.position.x;
			curY = f.position.y;
		 	curDistance = sqrt((pow(initX - curX, 2) + pow(initY - curY, 2))); //distance formula
		  
			ROS_INFO("%f, %f, %f, %f, \t %f",initX,initY,curX,curY,curDistance);
		
			if(curDistance < smallestDistance){ //find smallest distance
				smallestDistance = curDistance;
		  		index = i;
		  	}
		}
	  
		ROS_INFO("Chose closest pose:");

		return index;
	}

	//Loops through the points in the global point vector
	//sends goals to the navigation stack.
	bool sendGoals(double time){
		for(unsigned i = 0; i < targetPoints.size(); i++){
			seek(targetPoints[i], time);
		}
	}

	//gets the distance in x & y between the two given poses in meters
	double getDistance(geometry_msgs::Pose op1, geometry_msgs::Pose op2){
		double result;
		result = sqrt((pow(op1.position.x - op2.position.x, 2) + pow(op1.position.y - op2.position.y, 2))); //distance formula
		return std::abs(result);
	}


	//Sends goal to move_base. Moves the robot to the coordinates of the input PoseStamped
	bool seek(geometry_msgs::Pose& tp, double time){
	    move_base_msgs::MoveBaseGoal goal;
	    bool resetGoal = false; 
	    int goalsPassed = 0;
	    double distanceGoal = 1.0; //In meters
	    double curDistance = 0.0;
	    double oldDistance = 0.0;
	    bool cleared = false;
		ros::Duration dur(time); //The amount of time between goals that you want to pause
		geometry_msgs::Pose curLocation = curPosition;

	    goal.target_pose.header.stamp = ros::Time::now();
	    goal.target_pose.header.frame_id = "map";
	    goal.target_pose.pose.position.x = tp.position.x;
	    goal.target_pose.pose.position.y = tp.position.y;
	    goal.target_pose.pose.position.z = 0;
	    //get my current pose and preserve orientation
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	    ac_.sendGoal(goal);


		ros::Time start = ros::Time::now();
	    while(true){
	    	if(curDistance >= distanceGoal){
	    		ac_.cancelGoal(); //cancels the current goal
	    		ROS_INFO("Canceling goal at x: %f y:%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
	    		curLocation = curPosition;
	    		//ac_.waitForResult();
	    		// * make service call here ** //
	    		resetGoal = true;
	    	}
	    	if(ros::Time::now() - start >= dur){ //certain amount of time has passed...checks its location & distance
	    		getCurrentPose();
	    		oldDistance = curDistance;
	    		curDistance = getDistance(curPosition, curLocation);
	    	}
	    	if(ros::Time::now() - start >= ros::Duration(10.0) && (curDistance == oldDistance)){ //if the robot's distance hasn't changed in x seconds, move on to next goal
	    		//note: not sure if equals will work. may need to write function
	    		if(cleared){ //already cleared costmap. bigger issues. moves on to next goal
	    			ac_.cancelGoal();
	    			cleared = false;
	    			break; //bad policy. clean up.
	    		}
	    		else{
	    			ac_.cancelGoal();
	    			rotate(3.14); //rotates the robot, implicitly clearing the costmap.
	    			cleared = true;
	    		}
	    	}
	    	//if time needs to be reset: reset time and restart goal
	    	if(resetGoal){
	    		ROS_INFO("Pausing...");
	    		ros::Duration(2.0).sleep();
	    		//rotate(3.14);
	    		resetGoal = false;
	    		//the following lines should be cleaned up. currently a copy/paste from above
	    		goal.target_pose.header.stamp = ros::Time::now();
				ROS_INFO("Sending new goal.");


	    		start = ros::Time::now();
	    		ac_.sendGoal(goal);
	    	}
	    	ros::spinOnce();
	    	if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    		break;
	    }

	    if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	    	goalsPassed++;
	    	ROS_INFO("Goal reached. I've passed %d goals.", goalsPassed);
	    	return true;
	    }
	    else
	    	ROS_INFO("Failed to reach goal.");

	    return false;
	}
	void waitForGoalToCancel(){
		while(ac_.getState() != actionlib::SimpleClientGoalState::PREEMPTED)
			ros::spinOnce();
		ROS_INFO("Goal canceled.");
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "random_crawler");
	Crawler human_seeker(ros::this_node::getName());
	human_seeker.rearrangePoints(human_seeker.findClosestPose());
	human_seeker.sendGoals(1.0);
	return 0;
}
