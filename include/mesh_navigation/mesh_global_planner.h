#ifndef MESH_GLOBAL_PLANNER_H_
#define MESH_GLOBAL_PLANNER_H_

#include <path_planning/GraphHalfEdgeMesh.hpp>
#include <nav_core/base_global_planner.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <lvr_ros/lvr_ros_conversions.h>
#include <mesh_navigation/MeshGlobalPlannerConfig.h>

typedef lvr::ColorVertex<float, int> VertexType;
typedef lvr::Normal<float> NormalType;

namespace mesh_navigation
{

  class MeshGlobalPlanner : public nav_core::BaseGlobalPlanner{

    public:

      /**
       * @brief  Constructor
       */
      MeshGlobalPlanner();

      /**
       * @brief Constructor
       * @param mesh The mesh on which the planning works
       */
      MeshGlobalPlanner(mesh_msgs::TriangleMeshStamped& mesh);

      /**
       * @brief  Destructor
       */
      ~MeshGlobalPlanner();

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @param cost The plans calculated cost
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
          double& cost);

      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    protected:
      lvr::GraphHalfEdgeMesh<VertexType, NormalType> mesh;
      ros::Subscriber sub;
      ros::Publisher pub;
      void meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);
  };
} /* namespace mesh_navigation */

#endif /* mesh_global_planner.h */
