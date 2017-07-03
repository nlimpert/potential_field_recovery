/****************************************_*****************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <potential_field_recovery/potential_field_recovery.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(potential_field_recovery, PotentialFieldRecovery, potential_field_recovery::PotentialFieldRecovery, nav_core::RecoveryBehavior)

namespace potential_field_recovery {
PotentialFieldRecovery::PotentialFieldRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void PotentialFieldRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("min_dist", min_dist_, 0.4);

    blp_nh.param("max_trans_vel", max_trans_vel_, 0.3);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 0.2);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.05);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());


    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

PotentialFieldRecovery::~PotentialFieldRecovery(){
  delete world_model_;
}

void PotentialFieldRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the PotentialFieldRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("PotentialField recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_recovery", 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->resetLayers();
  ros::Duration(1).sleep();
  local_costmap_->getRobotPose(global_pose);

  double current_angle = -1.0 * M_PI;

  bool got_180 = false;

  double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  int id = 0;
  while(n.ok()){
    local_costmap_->getRobotPose(global_pose);

    double robot_x = global_pose.getOrigin().x();
    double robot_y = global_pose.getOrigin().y();
    double robot_ori = tf::getYaw(global_pose.getRotation());

    double footprint_cost = world_model_->footprintCost(robot_x, robot_y, robot_ori, local_costmap_->getRobotFootprint(), 0.0, 0.0);

    unsigned int width = local_costmap_->getCostmap()->getSizeInCellsX();
    unsigned int height = local_costmap_->getCostmap()->getSizeInCellsY();
    double cur_world_x = 0.0;
    double cur_world_y = 0.0;

    double cur_robot_x = global_pose.getOrigin().x();
    double cur_robot_y = global_pose.getOrigin().y();

    float target_x = 0.0, target_y = 0.0;
    float target_r = 0.0, target_phi = 0.0;

    float cur_min_dist = 100000.0;

    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0;
    points.color.a = 1.0;

    for (int posX = 0; posX < width; ++posX) {
      for (int posY = 0; posY < height; ++posY) {
        float cellCost = local_costmap_->getCostmap()->getCost(posX, posY);
        local_costmap_->getCostmap()->mapToWorld(posX, posY, cur_world_x, cur_world_y);
        if (local_costmap_->getCostmap()->getCost(posX, posY) != costmap_2d::FREE_SPACE) {
          double dx = cur_world_x - cur_robot_x;
          double dy = cur_world_y - cur_robot_y;

          geometry_msgs::Point p;
          p.x = cur_world_x;
          p.y = cur_world_y;
          p.z = 0.0;

          points.points.push_back(p);

          if (fabs(dx) >= 0.01 && fabs(dy) >= 0.01) {
            //Assign a lower factor for objects at larger distances
            float factor = 1.f / ( (dx*dx + dy*dy) * (dx*dx + dy*dy) );

            float d = sqrt( dx * dx + dy * dy );
            if (d < cur_min_dist) {
                cur_min_dist = d;
            }

            target_x -= factor * dx;
            target_y -= factor * dy;
          }
        }
      }
    }

    marker_pub.publish(points);
    points.points.clear();

    if (cur_min_dist >= min_dist_) {
        break;
    }

    target_phi = angles::normalize_angle(atan2(target_y, target_x) - robot_ori);

    float drive_part_x    = 0.f;
    float drive_part_y    = 0.f;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    drive_part_x = std::cos( target_phi );
    drive_part_y = std::sin( target_phi );

    cmd_vel.linear.x = drive_part_x * max_trans_vel_;
    cmd_vel.linear.y = drive_part_y * max_trans_vel_;

    vel_pub.publish(cmd_vel);
  }
}
};
