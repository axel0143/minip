#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <std_msgs/Int8.h>

using namespace cv;

double colPosX[4]={0,0,0,0}, colPosY[4];
ros::Publisher vis_pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void _send_goal(std_msgs::Int8 colSubbed)
{
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0))){ROS_INFO("Missing ROS nodes");}
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = colPosX[colSubbed.data];
	goal.target_pose.pose.position.y = colPosY[colSubbed.data];
	goal.target_pose.pose.orientation.w=1.0;
	ROS_INFO("Sending goal");
	while(true)
	{
		ac.sendGoal(goal);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Success");
			break;
		}
		else
		{
			ROS_INFO("Retrying");
			ros::Duration(0.5).sleep();
		}
	}
}

void _publish_marker(int colCount_2, double origin_x_1, double origin_y_1)
{
	double colBlue[4]={1, 0, 0.3, 0.2}, colGreen[4]={0, 0, 0.1, 1}, colRed[4]={0, 1, 0.6, 1};
	visualization_msgs::Marker marker;
	marker.ns = "minip";
	marker.header.frame_id = "map";
	marker.lifetime = ros::Duration();
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.001;
	marker.color.a = 1.0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;
	marker.pose.position.z = 0;
	marker.pose.position.x = origin_x_1;
	marker.pose.position.y = origin_y_1;
	marker.color.b = colBlue[colCount_2];
	marker.color.g = colGreen[colCount_2];
	marker.color.r = colRed[colCount_2];
	marker.id = colCount_2;
	vis_pub.publish(marker);
}

void _save_current_pos(int colCount_1)
{
	tf::TransformListener listener;
	double origin_x, origin_y;
  	while(true)
  	{
    	tf::StampedTransform transform;
    	try
    	{
        	listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
        	ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
        	origin_x = transform.getOrigin().x(), origin_y = transform.getOrigin().y();
        	break;
    	}
    	catch (tf::TransformException ex){ROS_ERROR("Nope! %s", ex.what());}
  	}
  	_publish_marker(colCount_1, origin_x, origin_y);
  	colPosX[colCount_1]=origin_x, colPosY[colCount_1]=origin_y;
  	for(int checkArr=0; checkArr !=-1; checkArr++)
	{
		if(colPosX[checkArr]==0){break;}
		if(checkArr==3)
		{
			ros::NodeHandle n;
			ros::Subscriber sub = n.subscribe("gui_goal", 1000, _send_goal);
			ros::spin();
		}
	}
	ros::Duration(0.5).sleep();
}

void _col_det ()
{
	int arrBlue[4] ={245, 90, 200, 60}, arrGreen[4]={165, 30, 100, 150}, arrRed[4]={40, 160, 140, 155};
	double percent[4];
	VideoCapture cap;
	if(!cap.open(1))
	{
	    ROS_INFO("Could not find camera");
	    return;
	}
	while(true)
	{
		for(int colCount=0; colCount<3; colCount++)
	  	{
	  		double pix_max=0;
	  		Mat frame;
	  		cap >> frame;
	  		if(frame.empty()){return;}
	  		for(int pix_rows=0; pix_rows<frame.rows; pix_rows++)
	  		{
	    		for(int pix_cols=0; pix_cols<frame.cols; pix_cols++)
	    		{
	      			Vec3b intensity = frame.at<Vec3b>(pix_rows, pix_cols);
	      			int blue = intensity.val[0], green = intensity.val[1], red = intensity.val[2];
			      	if (blue > arrBlue[colCount]-20 && blue < arrBlue[colCount]+20 &&
			      		green > arrGreen[colCount]-20 && green < arrGreen[colCount]+20 &&
			      		red > arrRed[colCount]-20 && red < arrRed[colCount]+20){pix_max++;}
	    		}
	  		}
			percent[colCount] = pix_max/(frame.rows*frame.cols)*100;
			if (percent[colCount]>5)
			{
				ROS_INFO("%f %%", percent[colCount]);
				_save_current_pos(colCount);
			}
		}
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "minip");
	ros::NodeHandle n;
	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
	_col_det();
}