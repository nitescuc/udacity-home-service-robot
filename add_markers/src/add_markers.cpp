#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;

bool goal_reached = false;
vector<double> last_position = { 0.0, 0.0 };

vector<double> pickup_position = { 2.0, 1.0 };
vector<double> drop_position = { -1.0, -1.0 };

ros::Publisher marker_pub;

void marker_action(ros::Publisher marker_pub, double x, double y, int action) 
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = action;

  // Set the pose of the marker (pickup).  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 0.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker_pub.publish(marker);
}

bool is_robot_at_position(vector<double> position)
{
  if (abs(last_position[0] - position[0]) < 0.2 && abs(last_position[1] - position[1]) < 0.2) return true;
  else return false;
}

void move_base_status_callback(const actionlib_msgs::GoalStatusArray status)
{
  //ROS_INFO("Status %d", status.status_list[0].status);
  switch (status.status_list[0].status) 
  {
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      if (!goal_reached) 
      {
        ROS_INFO("Goal reached by the robot. Last position (%1.02f, %1.02f)", last_position[0], last_position[1]);
        goal_reached = true;
        if (is_robot_at_position(pickup_position)) marker_action(marker_pub, pickup_position[0], pickup_position[1], visualization_msgs::Marker::DELETE);
        if (is_robot_at_position(drop_position)) marker_action(marker_pub, drop_position[0], drop_position[1], visualization_msgs::Marker::ADD);
      }
      break;
    default:
      goal_reached = false;
      break;
  }
}

void move_base_feedback_callback(const move_base_msgs::MoveBaseActionFeedback feedback)
{
  last_position[0] = feedback.feedback.base_position.pose.position.x;
  last_position[1] = feedback.feedback.base_position.pose.position.y;  
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  // Subscribe to status of the robot
  ros::Subscriber sub_status = n.subscribe("/move_base/status", 10, move_base_status_callback);
  ros::Subscriber sub_feedback = n.subscribe("/move_base/feedback", 10, move_base_feedback_callback);
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  // Display marker at pickup position
  marker_action(marker_pub, pickup_position[0], pickup_position[1], visualization_msgs::Marker::ADD);

  ros::spin();

  return 0;

}