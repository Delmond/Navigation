#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <iostream>
#include <chrono>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/*  ----------------------------*/

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>


#include <pluginlib/class_list_macros.h>


// Commend out if you dont want analytics logging
#define ANALYTICS

using namespace std;
using namespace std::chrono;

const double epsilon = numeric_limits<float>::epsilon();
const double infinity = numeric_limits<float>::infinity();

struct Node{
  float cost;
  int index;
};

class Astar: public nav_core::BaseGlobalPlanner{

    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2DROS* costmapROS;
    costmap_2d::Costmap2D* costmap;
    bool isInitilized;
    int width;
    int height;
    int mapSize;
    vector<bool> OGM;
    ros::Publisher guiPathPub;
  public:
    Astar(){
      isInitilized = false;
    }
    ~Astar(){}
    Astar(string name, costmap_2d::Costmap2DROS* costmap_ros){
      isInitilized = false;
      initialize(name, costmap_ros);
    }
    void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros){
      if(isInitilized){
         ROS_WARN("Astar global planner: Already initilized");
         return;
      }
      costmapROS = costmap_ros;
      costmap = costmap_ros->getCostmap();
      ros::NodeHandle nh("~/Astar");
      guiPathPub = nh.advertise<nav_msgs::Path>("plan", 1);

      isInitilized = true;
      ROS_INFO("Astar global planner: Initialized successfully");
    }
    const bool isInfinity(float x){
      return fabs(infinity - x) < epsilon;
    }
    const bool isInBounds(int x, int y){
      if( x < 0 || y < 0 || x >= height || y >= width)
        return false;
      return true;
    }
    const double getMoveCost(int firstIndex, int secondIndex) const{
      unsigned int tmp1, tmp2;
      costmap->indexToCells(firstIndex, tmp1, tmp2);
      int firstXCord = tmp1,firstYCord = tmp2;
      costmap->indexToCells(secondIndex, tmp1, tmp2);
      int secondXCord = tmp1, secondYCord = tmp2;

      int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
      // Error checking
      if(difference != 1 && difference != 2){
        ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
        return 1.0;
      }
      if(difference == 1)
        return 1.0;
      else
        return 1.4;
    }
    vector<int> getNeighborIndexes(int index){
      vector<int> neighborIndexes;
      for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++){
            unsigned tmp1, tmp2;
            costmap->indexToCells(index, tmp1, tmp2);

            int nextX = tmp1 + i;
            int nextY = tmp2 + j;
            int nextIndex = costmap->getIndex(nextX, nextY);
            if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex]){
              neighborIndexes.push_back(nextIndex);
            }
        }
      return move(neighborIndexes);
    }
    const double getHeuristic(int startCell, int goalCell) const{
      enum class Type { EUCLID, MANHATTAN };
      Type type = Type::EUCLID;
      unsigned int tmp1, tmp2;
      costmap->indexToCells(startCell, tmp1, tmp2);
      int startX = tmp1, startY = tmp2;
      costmap->indexToCells(goalCell, tmp1, tmp2);
      int goalX = tmp1, goalY = tmp2;

      if(type == Type::EUCLID){
        return sqrt(pow(goalY - startY, 2) + pow(goalX - startX, 2));
      }
      if(type == Type::MANHATTAN){
        return abs(goalY - startY) + abs(goalX - startX);
      }
    }

    bool makePlan(const geometry_msgs::PoseStamped& start,
    		const geometry_msgs::PoseStamped& goal,
    		std::vector<geometry_msgs::PoseStamped>& plan)
    {
      if(isInitilized == false){
        ROS_ERROR("Astart global planner: Initilize must be called first");
        return false;
      }
      costmap = costmapROS->getCostmap();
      width = costmap->getSizeInCellsX();
      height = costmap->getSizeInCellsY();
      mapSize = width * height;
      OGM.resize(mapSize);
      cout << "Doslo dovde!";
      for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++){
          int cost = costmap->getCost(j, i);
          OGM[i*width + j] = (cost == 0);
        }
      float startX = start.pose.position.x;
      float startY = start.pose.position.y;

      float goalX = goal.pose.position.x;
      float goalY = goal.pose.position.y;

      ROS_DEBUG("Astar global planner: Start cords (%.2f,%.2f) and goal cords (%.2f,%.2f)",
                startX, startY, goalX, goalY);
      plan.clear();
      // A* implementation
      vector<float> gCosts(mapSize, infinity);
      vector<int> cameFrom(mapSize, -1);
      // Acts as a priority queue
      multiset<Node> priority_costs;
      unsigned int tmp1, tmp2;
      costmap->worldToMap(startX, startY, tmp1, tmp2);
      int startIndex = costmap->getIndex(tmp1, tmp2);
      costmap->worldToMap(goalX, goalY, tmp1, tmp2);
      int goalIndex = costmap->getIndex(tmp1, tmp2);
      gCosts[startIndex] = 0;
      #ifdef ANALYTICS
      //** ANALYTICS **//
      int numberOfCellsVisited = 0;
      double pathLength = 0.0;
      int numberOfCellsInPath = 0;
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      //** END ANALYTICS **//
      #endif // ANALYTICS
      Node currentNode;
      currentNode.index = startIndex;
      currentNode.cost = gCosts[startIndex] + 0;
      priority_costs.insert(currentNode);
      while (!priority_costs.empty() && cameFrom[goalIndex] == -1){
        // Take the element from the top
        currentNode = *priority_costs.begin();
        //Delete the element from the top
        priority_costs.erase(priority_costs.begin());
        // Uzmi sve susjedne celije
        vector<int> neighborIndexes = getNeighborIndexes(currentNode.index);

        for(int i = 0; i < neighborIndexes.size(); i++){
          //cout << "neighborIndexes[i]: " << neighborIndexes[i] << endl;
          #ifdef ANALYTICS
          numberOfCellsVisited++;
          #endif // ANALYTICS
          if(cameFrom[neighborIndexes[i]] == -1){
            gCosts[neighborIndexes[i]] = gCosts[currentNode.index]
                  + getMoveCost(currentNode.index, neighborIndexes[i]);
            Node nextNode;
            nextNode.index = neighborIndexes[i];
            nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goalIndex);
            cameFrom[neighborIndexes[i]] = currentNode.index;
            priority_costs.insert(nextNode);
          }
        }
      }
      if(cameFrom[goalIndex] == -1){
        cout << "Goal not reachable, failed making a global path." << endl;
        return false;
      }
      #ifdef ANALYTICS
      //** ANALYTICS **//
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>( t2 - t1 ).count();
      //** END ANALYTICS **//
      #endif // ANALYTICS

      if(startIndex == goalIndex)
        return false;
      //Finding the best path
      vector<int> bestPath;
      currentNode.index = goalIndex;
      while(currentNode.index != startIndex){
         bestPath.push_back(cameFrom[currentNode.index]);
          currentNode.index = cameFrom[currentNode.index];
      }
      reverse(bestPath.begin(), bestPath.end());
      #ifdef ANALYTICS
      //** ANALYTICS **//
      numberOfCellsInPath = bestPath.size();
      //** END ANALYTICS **//
      #endif // ANALYTICS

      ros::Time plan_time = ros::Time::now();
      for(int i = 0; i < bestPath.size(); i++){
        unsigned int tmp1, tmp2;
        costmap->indexToCells(bestPath[i], tmp1, tmp2);
        double x, y;
        costmap->mapToWorld(tmp1,tmp2, x, y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = costmapROS->getGlobalFrameID();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        #ifdef ANALYTICS
        //** ANALYTICS **//
          if(i != 0){
            double dx = (plan[i - 1].pose.position.x - x);
            double dy = (plan[i - 1].pose.position.y - y);
            double segmentLength = sqrt(dx*dx + dy*dy);
            pathLength += segmentLength;
          }
          //** END ANALYTICS **//
        #endif // ANALYTICS
        plan.push_back(pose);
      }
      #ifdef ANALYTICS
      //** ANALYTICS **//
      cout << "ANALYTICS: " << endl;
      cout << "Time elapsed: " << duration << " microseconds" << endl;
      cout << "Number of cells visited: " << numberOfCellsVisited << endl;
      cout << "Path length: " << pathLength << endl;
      cout << "Number of cells in path: " << numberOfCellsInPath << endl;
      cout << "END ANALYTICS: " << endl;
      //** END ANALYTICS **//
      #endif // ANALYTICS
      publishGUIPath(plan);
      return true;
    }
    void publishGUIPath(std::vector<geometry_msgs::PoseStamped>& plan){
      nav_msgs::Path gui_path;
      gui_path.poses.resize(plan.size());
      if(!plan.empty())
      {
        gui_path.header.frame_id = plan[0].header.frame_id;
        gui_path.header.stamp = plan[0].header.stamp;
      }
      for(int i = 0; i < plan.size(); i++)
        gui_path.poses[i] = plan[i];

      guiPathPub.publish(gui_path);
    }
};
// Required for multiset sorting
bool operator <(const Node& x, const Node& y) {
  return x.cost < y.cost;
}

PLUGINLIB_EXPORT_CLASS(Astar, nav_core::BaseGlobalPlanner);
