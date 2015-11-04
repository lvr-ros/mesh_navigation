#ifndef MESH_LOCAL_PLANNER_H_
#define MESH_LOCAL_PLANNER_H_

#include "path_planning/GraphHalfEdgeMesh.hpp"
#include <robot_navigation/base_local_planner.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <lvr_ros/lvr_ros_conversions.h>
#include <mesh_navigation/MeshLocalPlannerConfig.h>


typedef lvr::ColorVertex<float, int> VertexType;
typedef lvr::Normal<float> NormalType;

namespace mesh_navigation{
  class MeshLocalPlanner : public robot_navigation::BaseLocalPlanner{

    public:

      /**
       * @brief  Constructor
       */
      MeshLocalPlanner();

      /**
       * @brief Constructor
       * @param mesh The mesh on which the planning works
       */
      MeshLocalPlanner(mesh_msgs::TriangleMeshStamped& mesh);

      /**
       * @brief  Destructor
       */
      ~MeshLocalPlanner();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param robot_pose The current global pose of the robot
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       */
      bool computeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Constructs the local planner
       * @param name The name to give this instance of the local planner
       */
      void initialize(std::string name);

    protected:
      lvr::GraphHalfEdgeMesh<VertexType, NormalType> mesh;
      ros::Subscriber sub;
      ros::Publisher pub;
      void meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);
  };
} /* namespace mesh_navigation */

#endif /* mesh_local_planner.h */
