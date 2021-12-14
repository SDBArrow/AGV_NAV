#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "std_msgs/String.h"
#include <move_base/move_base.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

ros::Publisher cmdVelPub;
ros::Publisher marker_pub;
geometry_msgs::Point current_point;
geometry_msgs::Pose pose_list[4];  //a pose consisting of a position and orientation in the map frame.

geometry_msgs::Point setPoint(double _x, double _y, double _z);
geometry_msgs::Quaternion setQuaternion(double _angleRan);
void init_goalList();
void shutdown(int sig);
void init_markers(visualization_msgs::Marker *marker);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal);
int destination;

void init_goalList()
{
	//How big is the square we want the robot to navigate?
	double square_size = 1.5;

	//Create a list to hold the target quaternions (orientations)
	geometry_msgs::Quaternion quaternions;
	geometry_msgs::Point point;

	point = setPoint(2.025, -0.869, 0.000);
	quaternions = setQuaternion( -3.123 );
	pose_list[1].position = point;
	pose_list[1].orientation = quaternions;

	point = setPoint(1.503, -5.940, 0.000);
	quaternions = setQuaternion( 1.573  );
	pose_list[2].position = point;
	pose_list[2].orientation = quaternions;

	point = setPoint(-1.314, -5.820, 0.000);
	quaternions = setQuaternion( 1.573 );
	pose_list[3].position = point;
	pose_list[3].orientation = quaternions;

	point = setPoint(-2.311, -0.350, 0.000);
	quaternions = setQuaternion( -0.003 );
	pose_list[4].position = point;
	pose_list[4].orientation = quaternions;
}

geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
	geometry_msgs::Point m_point;
	m_point.x = _x;
	m_point.y = _y;
	m_point.z = _z;
	return m_point;
}

geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
	geometry_msgs::Quaternion m_quaternion;
	m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan);
	return m_quaternion;
}

// Shutdown
void shutdown(int sig)
{
	cmdVelPub.publish(geometry_msgs::Twist());
	ros::Duration(1).sleep(); // sleep for  a second
	ROS_INFO("move_base_square_agv.cpp ended!");
	ros::shutdown();
}

// Init markers
void init_markers(visualization_msgs::Marker *marker)
{
	marker->ns = "waypoints";
	marker->id = 0;
	marker->type = visualization_msgs::Marker::CUBE_LIST;
	marker->action = visualization_msgs::Marker::ADD;
	marker->lifetime = ros::Duration();//0 is forever
	marker->scale.x = 0.2;
	marker->scale.y = 0.2;
	marker->color.r = 1.0;
	marker->color.g = 0.7;
	marker->color.b = 1.0;
	marker->color.a = 1.0;

	marker->header.frame_id = "map";
	marker->header.stamp = ros::Time::now();
}


// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal Received");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
//	ROS_INFO("Got base_position of Feedback");
	current_point.x = feedback->base_position.pose.position.x;
	current_point.y = feedback->base_position.pose.position.y;
	current_point.z = feedback->base_position.pose.position.z;
}

double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal)
{
	double m_distance;
	m_distance = sqrt(pow(fabs(m_goal.x - m_current_point.x), 2) + pow(fabs(m_goal.y - m_current_point.y), 2));
	return m_distance;
}

void dataCallback(const std_msgs::String::ConstPtr& msg)
{
	destination = stoi(msg->data);
	ROS_INFO("%d", destination);
	Client ac("move_base", true);
	//初始化航點目標
	move_base_msgs::MoveBaseGoal goal;

	//Use the map frame to define goal poses
	goal.target_pose.header.frame_id = "map";

	//Set the time stamp(標記) to "now"
	goal.target_pose.header.stamp = ros::Time::now();
        
        if ( destination != 0 && destination < 5){
		//清空之前的導航目標
		ros::NodeHandle node;
		ros::Publisher cancle_pub_ = node.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
		actionlib_msgs::GoalID first_goal;
		cancle_pub_.publish(first_goal);
		//重置障礙層
		ros::ServiceClient client = node.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
       		std_srvs::Empty srv;
		client.call(srv);
		//設定目的地
		goal.target_pose.pose = pose_list[destination];

		//Start the robot moving toward the goal
		ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
	}
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_move_base");
	ros::NodeHandle node;

	ros::Rate loop_rate(10);

	//Subscribe to the move_base action server
	Client ac("move_base", true);

	//Define a marker publisher.
	marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

	signal(SIGINT, shutdown);
	//ROS_INFO("move_base_square.cpp start...");

	//初始化 the list of goal
	init_goalList();

	//for init_markers function
	visualization_msgs::Marker  marker_list;

	//初始化RViz的可視化標記
	init_markers(&marker_list);

	//在每個航路點設置可視化標記
	for (int i = 0; i < 4; i++)
	{
		marker_list.points.push_back(pose_list[i].position);
	}

	ROS_INFO("Waiting for move_base action server...");

	//等待action server60秒
	if (!ac.waitForServer(ros::Duration(60)))
	{
		ROS_INFO("Can't connected to move base server");
		return 1;
	}
	
	//接收控制訊息 
	ros::Subscriber sub = node.subscribe("/server_messages", 1000, dataCallback);

        ros::spin();
	return 0;
}
