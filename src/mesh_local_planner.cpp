#include "mesh_navigation/mesh_local_planner.h"

namespace mesh_navigation{

  MeshLocalPlanner::MeshLocalPlanner(){

  }

  MeshLocalPlanner::MeshLocalPlanner(mesh_msgs::TriangleMeshStamped& mesh){

  }

  MeshLocalPlanner::~MeshLocalPlanner(){}

  bool MeshLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

    return false;
  }


  bool MeshLocalPlanner::isGoalReached(){
    return false;
  }


  bool MeshLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){

  }

  void MeshLocalPlanner::initialize(
    std::string name,
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros
  ){

    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle nh_g;
    sub = nh_g.subscribe("mesh_in", 1, &MeshLocalPlanner::meshCallback, this);
    pub = nh_g.advertise<mesh_msgs::TriangleMeshStamped>("planning_mesh", 1);

  }

  void MeshLocalPlanner::meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg){
    lvr::MeshBufferPtr mesh_buffer_ptr(new lvr::MeshBuffer);
    lvr_ros::fromTriangleMeshToMeshBuffer(mesh_msg->mesh, *mesh_buffer_ptr);
    mesh = lvr::GraphHalfEdgeMesh<VertexType, NormalType>(mesh_buffer_ptr);
  }

} /* namespace mesh_navigation */
