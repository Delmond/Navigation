#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdio.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
/*  ----------------------------*/

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <pluginlib/class_list_macros.h>


// CONSTANTS
const double PI                          = 3.141592653589793238463;
const double MAX_LINEAR_VELOCITY         = 1.0;
const double MIN_LINEAR_VELOCITY         = 0.0;
const double MAX_ANGULAR_VELOCITY        = PI*30.0/180.0;
const double MIN_ANGULAR_VELOCITY        = -PI*30.0/180.0;
const double MAX_LINEAR_ACCELERATION     = 2.0;
const double MIN_LINEAR_ACCELERATION     = -2.0;
const double MAX_ANGULAR_ACCELERATION    = PI*30.0/180.0;
const double MIN_ANGULAR_ACCELERATION    = -PI*30.0/180.0;
const double DT                          = 0.5;
const double ROBOT_RADIUS                = 0.05;
// const double WEIGHT_HEADING = 0.05;
// const double WEIGHT_CLEARANCE = 0.2;
// const double WEIGHT_VELOCITY = 0.1;
const double WEIGHT_HEADING = 0.2;
const double WEIGHT_CLEARANCE = 6;
const double WEIGHT_VELOCITY = 0.2;
const double EPSILON = std::numeric_limits<double>::epsilon();
const int stepsAhead = 10;

using namespace std;

class DynamicWindow: public nav_core::BaseLocalPlanner {

  bool initialized;
  tf2_ros::Buffer* tf;
  costmap_2d::Costmap2DROS* costmap_ros;
  costmap_2d::Costmap2D* costmap;
  int width;
  int height;
  int mapSize;
  geometry_msgs::PoseStamped currentPose;
  geometry_msgs::PoseStamped goalPose;
  vector<geometry_msgs::PoseStamped> globalPlan;
  ros::Subscriber odometry_sub;
  double currentV;
  double currentW;
  ros::Publisher guiPathPub;
  ros::Publisher waypointPub;
  nav_msgs::Path gui_path;
  struct ScoringHelper{
    double clearance;
    double heading;
    double v;
    double w;
    double score;
  };
public:
  DynamicWindow(): initialized(false) {}
  double getClearnace(geometry_msgs::PoseStamped robotPose){
    double minDistance = 100000;
    // costmap->worldToMap(robotPose.pose.position.x, robotPose.pose.position.y, CordX, CordY);
    for(int i = 0; i < costmap->getSizeInCellsX(); i++){
      for(int j = 0; j < costmap->getSizeInCellsY(); j++){

        //int newIndex = costmap->getIndex(CordX + i, CordY + j);
        if(DynamicWindow::costmap->getCost(i,j) == 0){
          continue;
        }
        double obstacleX, obstacleY;
        costmap->mapToWorld(i, j, obstacleX, obstacleY);
        double newDistance = sqrt(pow(robotPose.pose.position.x - obstacleX, 2) + pow(robotPose.pose.position.y - obstacleY,2)) - ROBOT_RADIUS;
        if(newDistance < minDistance){
           minDistance = newDistance;
        }
      }
    }
    if(minDistance > 2 * ROBOT_RADIUS){
      minDistance = 2 * ROBOT_RADIUS;
    }
    return minDistance;
  }
  double getClearnaceAlternative(geometry_msgs::PoseStamped robotPose, double velocity, double rotationalVelocity){
    double minDistance = 100000;
    // costmap->worldToMap(robotPose.pose.position.x, robotPose.pose.position.y, CordX, CordY);
    double distanceToObstacle = 0.0;
    bool obstacleFound = false;
    for(int i = 0; i < 20; i++){
        geometry_msgs::PoseStamped newRobotPose = getNewRobotPose(robotPose, velocity, rotationalVelocity);

        double dx = newRobotPose.pose.position.x - robotPose.pose.position.x;
        double dy = newRobotPose.pose.position.y - robotPose.pose.position.y;
        distanceToObstacle += sqrt(dx*dx + dy*dy);

        unsigned int tmp1, tmp2;
        costmap->worldToMap(newRobotPose.pose.position.x, newRobotPose.pose.position.y, tmp1, tmp2);

        if(DynamicWindow::costmap->getCost(tmp1, tmp2) != 0){
          obstacleFound = true;
          break;
        }
        robotPose = newRobotPose;
      }
      if(obstacleFound){
        return (distanceToObstacle > 2.0)? 2.0 : distanceToObstacle;
      } else{
        return 2.0;
      }
  }
  double desiredVelocity(geometry_msgs::PoseStamped robotPose){
    double dx = robotPose.pose.position.x - goalPose.pose.position.x;
    double dy = robotPose.pose.position.y - goalPose.pose.position.y;
    double distanceToGoal = sqrt(dx*dx + dy*dy);
    return distanceToGoal*DT;
  }
  double getHeading(geometry_msgs::PoseStamped robotPose){
    double angleToGoal = atan2(goalPose.pose.position.y - robotPose.pose.position.y, goalPose.pose.position.x - robotPose.pose.position.x );

      // cout << "Angle to goal is: " << angleToGoal << " [rad], ";
    angleToGoal = angles::normalize_angle((angles::normalize_angle_positive(angleToGoal) - angles::normalize_angle_positive(tf::getYaw(robotPose.pose.orientation))));
    // cout << "difference in angles of robot and angle to goal is: " << angleToGoal <<" [rad], " <<  180 -  abs(angleToGoal)/PI * 180 <<endl;

    return 180 -  abs(angleToGoal)/PI * 180;
    // return abs(angleToGoal)/PI * 180;
  }
  double getBreakingDistance(double velocity){
    double breakingDistance = 0.0;
    while(velocity > 0){
      breakingDistance += velocity*DT;
      velocity = velocity + MIN_LINEAR_ACCELERATION*DT;
    }
    return breakingDistance;
  }
  geometry_msgs::PoseStamped getNewRobotPose(geometry_msgs::PoseStamped robotPose, double velocity, double rotationalVelocity){
    geometry_msgs::PoseStamped newRobotPose;
    double robotYaw = tf::getYaw(robotPose.pose.orientation);
    newRobotPose.pose.position.x = robotPose.pose.position.x + velocity*DT*cos(robotYaw);
    newRobotPose.pose.position.y = robotPose.pose.position.y + velocity*DT*sin(robotYaw);
    newRobotPose.pose.position.z = 0;
    double newRobotYaw = robotYaw + rotationalVelocity * DT;
    newRobotPose.pose.orientation = tf::createQuaternionMsgFromYaw(newRobotYaw);
    return newRobotPose;
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
      currentV = odom->twist.twist.linear.x;
      currentW = odom->twist.twist.angular.z;
  }
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(initialized){
      ROS_WARN("DW local planner: Already initilized");
      return;
    }
    DynamicWindow::tf = tf;
    DynamicWindow::costmap_ros = costmap_ros;
    costmap = costmap_ros->getCostmap();
    costmap_ros->getRobotPose(currentPose);
    width = costmap->getSizeInCellsX();
    height = costmap->getSizeInCellsY();
    mapSize = width * height;
    ROS_INFO("Width is: %d, Height is: %d", width, height);

    ros::NodeHandle nh("~/DynamicWindow");
    ros::NodeHandle oh;
    odometry_sub = oh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&DynamicWindow::odomCallback, this, _1));
    guiPathPub = nh.advertise<nav_msgs::Path>("plan", 1);
    waypointPub = nh.advertise<nav_msgs::Path>("waypoints", 1);

    initialized = true;

  }
  geometry_msgs::PoseStamped getNewRobotGoal(geometry_msgs::PoseStamped robotPose){
    int closestPointInPath = 0;
    double shortestDistance = 1000000;
    for(int i = 0; i < globalPlan.size(); i++){
      double dx = globalPlan[i].pose.position.x - robotPose.pose.position.x;
      double dy = globalPlan[i].pose.position.y - robotPose.pose.position.y;
      double newDistance = sqrt(dx*dx + dy*dy);
      if(newDistance < shortestDistance){
        shortestDistance = newDistance;
        closestPointInPath = i;
      }
    }
    if(globalPlan.size() - closestPointInPath < stepsAhead){
      return globalPlan.back();
    }
    return globalPlan[closestPointInPath + stepsAhead];
  }
  bool setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan){
    globalPlan = orig_global_plan;
    goalPose = orig_global_plan.back();
    gui_path.poses.clear();
    return true;
  }
  const double getLinearVelocity() const{
    return currentV;
  }
  const double getAngularVelocity() const{
    return currentW;
  }
  vector<geometry_msgs::PoseStamped> getLocalPlan(geometry_msgs::PoseStamped robotPose, double velocity, double rotationalVelocity){
    vector<geometry_msgs::PoseStamped> localPlan;
      robotPose = getNewRobotPose(robotPose, velocity, rotationalVelocity);
      localPlan.push_back(robotPose);
    return localPlan;
  }

  double clamp(double value, double minimal, double maximal){
    if(value < minimal)
      return minimal;
    if(value > maximal)
      return maximal;
    return value;
  }

  bool computeVelocityCommands (geometry_msgs::Twist &cmd_vel){
    if(!initialized){
      ROS_WARN("DW local planner: Must be initilized first");
      return false;
    }
    if(costmap_ros->getRobotPose(currentPose) == false){
      ROS_WARN("DW local planner: Failed to get current local pose");
      return false;
    }
    costmap = costmap_ros->getCostmap();
    costmap_ros->getRobotPose(currentPose);
    width = costmap->getSizeInCellsX();
    height = costmap->getSizeInCellsY();
    mapSize = width * height;
    ROS_INFO("Width is: %d, Height is: %d", width, height);


    double robotV = getLinearVelocity();
    double robotW = getAngularVelocity();
//  double desiredV = calculateDesiredVelocity(currentPose, goalPose);

    vector<ScoringHelper> scoringhelper;
    goalPose = getNewRobotGoal(currentPose);
    double minV = clamp(robotV + DT*MIN_LINEAR_ACCELERATION, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxV = clamp(robotV + DT*MAX_LINEAR_ACCELERATION, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);

    double minW = clamp(robotW + DT*MIN_ANGULAR_ACCELERATION, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    double maxW = clamp(robotW + DT*MAX_ANGULAR_ACCELERATION, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    ROS_INFO("RobotW: %4lf, DT: %4lf, MIN_ANGULAR_ACCELERATION: %4lf",robotW,DT, MIN_ANGULAR_ACCELERATION);
    double velocityPrecision = 0.01;
    double angularPrecision = 5/180.0 * PI;

    ROS_INFO("DW Velocity range (%4f,%4f), rotational range (%4f, %4f)", minV,maxV, minW,maxW);
    double sumHeading = 0.0, sumClearance = 0.0, sumVelocity = 0.0;
    for(double v = minV; v <= maxV; v += velocityPrecision){
        for(double w = minW; w <= maxW; w += angularPrecision){
          vector<geometry_msgs::PoseStamped> localPlan = getLocalPlan(currentPose, v, w);
          // double clearance = getClearnace(localPlan.back());
          double clearance = getClearnaceAlternative(currentPose, v, w);
          double breakingDistance = getBreakingDistance(v);
          if(clearance <= breakingDistance)
            continue;

          double heading = getHeading(localPlan.back());
          sumHeading += heading;
          sumClearance += clearance;
          sumVelocity += v;

          ScoringHelper sh;
          sh.clearance = clearance;
          sh.heading = heading;
          // sh.v = clamp(v, 0.0, desiredVelocity(localPlan.back()) );
          sh.v = v;
          sh.w = w;
          scoringhelper.push_back(sh);
      }
    }
    int bestScoreIndex = 0;
    for(int i = 0; i < scoringhelper.size(); i++){
      double normalized_heading = scoringhelper[i].heading/sumHeading;
      double normalized_clearance = scoringhelper[i].clearance/sumClearance;
      double normalized_velocity = (sumVelocity < EPSILON)? 0 : scoringhelper[i].v/sumVelocity;
      scoringhelper[i].score =
            WEIGHT_HEADING * normalized_heading +
            WEIGHT_CLEARANCE * normalized_clearance +
            WEIGHT_VELOCITY * normalized_velocity;
       // scoringhelper[i].score =
       //       WEIGHT_HEADING * scoringhelper[i].heading +
       //       WEIGHT_CLEARANCE * scoringhelper[i].clearance +
       //       WEIGHT_VELOCITY * scoringhelper[i].v;
      printf("Score for %d with (v: %lf, w: %lf, heading: %lf, clearance: %lf ) is %lf \n", i, scoringhelper[i].v, scoringhelper[i].w,
      scoringhelper[i].heading, scoringhelper[i].clearance, scoringhelper[i].score);
         if(scoringhelper[i].score >= scoringhelper[bestScoreIndex].score)
           bestScoreIndex = i;
    }
    if(scoringhelper.size() == 0){
      cout << "Failed finding velocity comands" << endl;
      return false;
    }
    cmd_vel.linear.x = scoringhelper[bestScoreIndex].v;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = scoringhelper[bestScoreIndex].w;

    vector<geometry_msgs::PoseStamped> lplan = getLocalPlan(currentPose,scoringhelper[bestScoreIndex].v,scoringhelper[bestScoreIndex].w);
    publishWaypoint(currentPose);
    return true;
  }
  bool isGoalReached(){
    if(!initialized){
      ROS_WARN("DW local planner: Must be initilized first");
      return false;
    }
    if(costmap_ros->getRobotPose(currentPose) == false){
      ROS_WARN("DW local planner: Failed to get current local pose");
      return false;
    }
    geometry_msgs::PoseStamped finalGoal = globalPlan.back();
    double currentXCord = currentPose.pose.position.x;
    double currentYCord = currentPose.pose.position.y;
    double goalXCord = finalGoal.pose.position.x;
    double goalYCord = finalGoal.pose.position.y;

    // compute distance from goal position
    double positionalDistance = sqrt(pow(currentXCord - goalXCord, 2) + pow(currentYCord - goalYCord,2));
    // compute distance from rotational positions
    double currentYaw = angles::normalize_angle_positive(tf2::getYaw(currentPose.pose.orientation));
    double goalYaw = angles::normalize_angle_positive(tf2::getYaw(finalGoal.pose.orientation));
    double rotationalDistance = abs(currentYaw - goalYaw);

    if(positionalDistance < 0.1)
      return true;

    // if(positionalDistance < 0.1 && rotationalDistance < 5*PI/180.0)
    //   return true;

    return false;
  }
  void publishWaypoint(geometry_msgs::PoseStamped waypoint){
    ros::Time plan_time = ros::Time::now();
    gui_path.poses.resize(gui_path.poses.size() + 1);
    gui_path.header.frame_id = costmap_ros->getGlobalFrameID();
    gui_path.header.stamp = plan_time;

    gui_path.poses[gui_path.poses.size() -1] = waypoint;
    gui_path.poses[gui_path.poses.size() -1].header = gui_path.header;

    guiPathPub.publish(gui_path);
  }

};
PLUGINLIB_EXPORT_CLASS(DynamicWindow, nav_core::BaseLocalPlanner);
