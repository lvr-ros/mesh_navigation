#include "mesh_navigation/mesh_global_planner.h"


namespace mesh_navigation{

  MeshGlobalPlanner::MeshGlobalPlanner(){

  }

  MeshGlobalPlanner::MeshGlobalPlanner(mesh_msgs::TriangleMeshStamped& mesh){

  }

  MeshGlobalPlanner::~MeshGlobalPlanner(){}

  bool MeshGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    double cost;
    return makePlan(start, goal, plan, cost);
  }

  bool MeshGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
      double& cost){


    return true;
  }

  void MeshGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle nh_g;
    sub = nh_g.subscribe("mesh_in", 1, &MeshGlobalPlanner::meshCallback, this);
    pub = nh_g.advertise<mesh_msgs::TriangleMeshStamped>("planning_mesh", 1);

    //nh.param("", var, default_value);

  }

  void MeshGlobalPlanner::meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg){
    lvr::MeshBufferPtr mesh_buffer_ptr(new lvr::MeshBuffer);
    lvr_ros::fromTriangleMeshToMeshBuffer(mesh_msg->mesh, *mesh_buffer_ptr);
    mesh = lvr::GraphHalfEdgeMesh<VertexType, NormalType>(mesh_buffer_ptr);
  }

} /* namespace mesh_navigation */
