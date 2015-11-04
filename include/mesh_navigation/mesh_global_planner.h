#ifndef MESH_GLOBAL_PLANNER_H_
#define MESH_GLOBAL_PLANNER_H_

#include "path_planning/GraphHalfEdgeMesh.hpp"
#include <robot_navigation/base_global_planner.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <lvr_ros/lvr_ros_conversions.h>
#include <mesh_navigation/MeshGlobalPlannerConfig.h>
#include <tf/transform_listener.h>

typedef lvr::ColorVertex<float, int> VertexType;
typedef lvr::Normal<float> NormalType;

namespace mesh_navigation
{

  class MeshGlobalPlanner : public robot_navigation::BaseGlobalPlanner{

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
       * @param tolerance The goal tolerance
       * @param plan The plan... filled by the planner
       * @param cost The costs for the plan 
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(
		const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal,
        const double& tolerance,
        std::vector<geometry_msgs::PoseStamped>& plan,
        double& costs);

      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       */
      void initialize(const std::string& name, const boost::shared_ptr<tf::TransformListener>& tf_listener, const std::string& global_frame);

    protected:
      
      void msgPointToVertex(const geometry_msgs::Point& point, VertexType& vertex);

      void meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);
      
      
      /**
       * @brief calculates the pose at the current vertex 
       * @param current The current vertex in the path 
       * @param next The next vertex in the path
       * @param normal The normal of the underlying surface (face or vertex normal)
       * @param pose The calculated pose
       * @param return true if the pose is available
       */
      bool calculatePose(
        const VertexType& current,
        const VertexType& next,
        const NormalType& normal,
        geometry_msgs::Pose& pose
      );

      lvr::GraphHalfEdgeMesh<VertexType, NormalType>::Ptr mesh_ptr;
      ros::Subscriber sub;
      ros::Publisher navigation_mesh_pub;
      ros::Publisher planning_result_pub;
      
      std::string global_frame;
      boost::shared_ptr<tf::TransformListener> tf_listener_ptr;
      double tf_timeout;
    
      double edge_region_growing_max_dist_;
      double edge_region_growing_max_normal_angle_;
      
      enum GraphBaseType{
        VertexGraph,
        FaceGraph
      };
      
      GraphBaseType graph_base_type_;
	  
  };
} /* namespace mesh_navigation */

#endif /* mesh_global_planner.h */
