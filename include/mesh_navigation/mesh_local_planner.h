#ifndef MESH_LOCAL_PLANNER_H_
#define MESH_LOCAL_PLANNER_H_

#include "path_planning/GraphHalfEdgeMesh.hpp"
#include <nav_core/base_local_planner.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <lvr_ros/lvr_ros_conversions.h>
#include <mesh_navigation/MeshLocalPlannerConfig.h>


typedef lvr::ColorVertex<float, int> VertexType;
typedef lvr::Normal<float> NormalType;

namespace mesh_navigation{
  class MeshLocalPlanner : public nav_core::BaseLocalPlanner{

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
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

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
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
      void initialize(
        std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros);

    protected:
      lvr::GraphHalfEdgeMesh<VertexType, NormalType> mesh;
      ros::Subscriber sub;
      ros::Publisher pub;
      void meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);
  };
} /* namespace mesh_navigation */

#endif /* mesh_local_planner.h */
