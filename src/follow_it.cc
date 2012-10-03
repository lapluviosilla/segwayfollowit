#include <ros/ros.h>

#include <vector>
#include <cmvision/Blobs.h>
#include <geometry_msgs/Twist.h>
#include <boost/foreach.hpp>

namespace {
  // constants
  const int x_threshold_pct = 15;
  const float angular_velocity = 0.3;
  const float linear_velocity = 0.4;
  const int area_max = 1800;
  const int area_min = 200;
    // ideal distance
  const int target_area = 900;
  const int target_area_threshold = 80;

  // publishers
  ros::Publisher pub_velocities;

  // current velocities
  float vel_current_linear_x;
  float vel_current_angular_z;

  bool is_ball_there;
  float ball_gone_time; 
}

void calculateVelocities(const ros::TimerEvent&) {
  // For now just used to kill linear velocity if blob disappeared
  if (!is_ball_there) {
    ball_gone_time += 0.5;
    vel_current_linear_x = 0.0;
    // If ball is gone for more then two seconds, then look for it for 12 seconds
    if (ball_gone_time > 2 && ball_gone_time < 14) {
      vel_current_angular_z = angular_velocity;
    } else {
      vel_current_angular_z = 0.0;
    }
  }  

  //if (vel_current_angular_z > 0) {
  //  vel_current_angular_z 
  //  vel_current_angular_z -= 0.02;
  //}
}

void publishVelocities() {
  geometry_msgs::TwistPtr outMsg(new geometry_msgs::Twist);
  outMsg->linear.x = vel_current_linear_x;
  outMsg->angular.z = vel_current_angular_z;
  ROS_INFO_STREAM("Linear Vel: " << vel_current_linear_x << " Angular vel: " << vel_current_angular_z);
  pub_velocities.publish(outMsg);
}

void blobUpdate(cmvision::Blobs::ConstPtr msg) {
  int blob_count = msg->blob_count;
  int image_width = msg->image_width;
  int image_height = msg->image_height;

  int mid_point_x = image_width / 2;
  int x_threshold_px = image_width * x_threshold_pct / 100;

  // Which are the "valid" blobs?
  std::vector<cmvision::Blob> valid_blobs;
  BOOST_FOREACH( cmvision::Blob blob, msg->blobs) {
    if (blob.area < area_max && blob.area > area_min) {
      valid_blobs.push_back(blob);
    }
  }
  // Make sure we only have 1 valid blob (the orange ball)
  if(valid_blobs.size() == 1) {
    cmvision::Blob o_blob = valid_blobs[0];
    // ROS_INFO_STREAM("we may proceed, ball is at x: " << o_blob.x);
    // Make sure we have a blob of aproximate ball size to avoid other unwanted objects
    ROS_INFO("tracking");
    is_ball_there = true;
    ball_gone_time = 0.0;
    if (o_blob.x < (mid_point_x - x_threshold_px)) {
      ROS_INFO("GO LEFT");
      vel_current_angular_z = angular_velocity;
    } else if (o_blob.x > (mid_point_x + x_threshold_px)) {
      ROS_INFO("GO RIGHT");
      vel_current_angular_z = -1 * angular_velocity;
    } else {
      vel_current_angular_z = 0.0;
    }
    // If small then go forward, if too large then go back.
    if (o_blob.area < (target_area - target_area_threshold)) {
      vel_current_linear_x = linear_velocity;
    } else if (o_blob.area > (target_area + target_area_threshold)) {
      vel_current_linear_x = -1 * linear_velocity;
    } else {
      vel_current_linear_x = 0.0;
    }
  } else {
    is_ball_there = false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "p4_example_sub");

  ROS_INFO("Follower started");

  ros::NodeHandle n;

  ros::Subscriber sub_blobs = 
      n.subscribe<cmvision::Blobs>("blobs", 1000, blobUpdate);
  
  pub_velocities = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Timer timer = n.createTimer(ros::Duration(0.5), calculateVelocities);
 
  while(ros::ok()) {
    publishVelocities();
    ros::spinOnce();
  }
}
