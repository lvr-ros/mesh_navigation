#include "mesh_navigation/mesh_global_planner.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <robot_navigation_state/robot_navigation_state.h>
#include <mesh_msgs_transform/transforms.h>

namespace mesh_navigation{

  MeshGlobalPlanner::MeshGlobalPlanner(){

  }


  MeshGlobalPlanner::~MeshGlobalPlanner(){}

  bool MeshGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    const double& tolerance,
    std::vector<geometry_msgs::PoseStamped>& plan,
    double& cost
  ){
    if(mesh_ptr){
      plan.clear();
    
      geometry_msgs::PoseStamped start_point, goal_point;
      
      // start and goal should already be in the global frame
      // transform to global frame, if not - the mesh is the global_frame
      if(!robot_navigation_state::transformPose(
          *tf_listener_ptr,
          global_frame,
          start_point.header.stamp,
          ros::Duration(tf_timeout),
          start,
          global_frame,
          start_point
        )
      ){
          return false;
      }
      if(!robot_navigation_state::transformPose(
          *tf_listener_ptr,
          global_frame,
          goal_point.header.stamp,
          ros::Duration(tf_timeout), 
          goal,
          global_frame,
          goal_point
        )
      ){
          return false;
      }

      VertexType start_vertex;
      msgPointToVertex(start_point.pose.position, start_vertex);
      
      VertexType goal_vertex;
      msgPointToVertex(goal_point.pose.position, goal_vertex);
        
      std::vector<VertexType> path;
      std::vector<NormalType> path_normals;
      ROS_INFO("global start point: (%f %f %f)", start_vertex.x, start_vertex.y, start_vertex.z);
      ROS_INFO("global goal point: (%f %f %f)", goal_vertex.x, goal_vertex.y, goal_vertex.z);
      
      float roughness_factor = 3;
      float height_diff_factor = 3;
      
      
      switch(graph_base_type_){
        case VertexGraph:
		      mesh_ptr->vertexGraphCalculateEdgeWeights(roughness_factor, height_diff_factor);
          if(!mesh_ptr->vertexGraphDijkstra(start_vertex, goal_vertex, path, path_normals)){
            return false;  
          }
          break;
        case FaceGraph:
   		    mesh_ptr->faceGraphCalculateEdgeWeights(roughness_factor, height_diff_factor);
          if(!mesh_ptr->faceGraphDijkstra(start_vertex, goal_vertex, path, path_normals)){
            return false;  
          }
          break;
      }
      
      std::vector<NormalType>::iterator normal_iter;
      std::vector<VertexType>::iterator vertex_iter;
      
      // add the goal vertex to the path to make a pose calculation
      // available for the last vertex in the path
      path.push_back(goal_vertex);
      
      // iterate till the penultimate vertex path.end()-1 
      for(
        vertex_iter = path.begin(), normal_iter = path_normals.begin();
        vertex_iter != path.end()-1; ++vertex_iter)
      {
        // calculate the pose by using the direction to the next vertex
        geometry_msgs::PoseStamped pose_msg;
        if(calculatePose(*vertex_iter, *(vertex_iter+1), *normal_iter, pose_msg.pose)){
          pose_msg.header.stamp = start_point.header.stamp;
          pose_msg.header.frame_id = global_frame;
          plan.push_back(pose_msg);
        } 
      }
      
      
      // prepare mesh message
      mesh_msgs::TriangleMeshStamped result_mesh_msg;
      result_mesh_msg.header.stamp = ros::Time::now();
      result_mesh_msg.header.frame_id = global_frame;

      lvr_ros::fromMeshBufferToTriangleMesh(mesh_ptr->meshBuffer(), result_mesh_msg.mesh);
      
      ROS_INFO("Add navigation colors...");
      
      std::vector<float> face_costs, vertex_costs;
      int goal_index;
      
      switch(graph_base_type_){
        case VertexGraph:
          mesh_ptr->getDistancesVertexGraph(vertex_costs);
          mesh_ptr->getNearestVertexIndexVertexGraph(goal_vertex, goal_index);
          lvr_ros::intensityToVertexRainbowColors(vertex_costs, result_mesh_msg.mesh, 0, vertex_costs[goal_index]);
          break;
        case FaceGraph:
          mesh_ptr->getDistancesFaceGraph(face_costs);
          mesh_ptr->getNearestVertexIndexFaceGraph(goal_vertex, goal_index);
          lvr_ros::intensityToTriangleRainbowColors(face_costs, result_mesh_msg.mesh, 0, face_costs[goal_index]);
          break;
      }

      ROS_INFO("Publish planning result mesh!");
      planning_result_pub.publish(result_mesh_msg);
      
      return true;
    }else{
      return false;
    } 
  }
  
  void MeshGlobalPlanner::msgPointToVertex(const geometry_msgs::Point& point, VertexType& vertex){
  vertex.x = point.x;
  vertex.y = point.y;
  vertex.z = point.z;  
  }
  
  
  bool MeshGlobalPlanner::calculatePose(const VertexType& current, const VertexType& next, const NormalType& normal, geometry_msgs::Pose& pose){
  
  VertexType direction = next - current;
  VertexType null_vector;
  if(direction == null_vector){
    return false;
    ROS_ERROR("The direction vector is null!");
  }
    NormalType z(normal);    
    NormalType y(normal.cross(direction));
    NormalType x(y.cross(normal));
    ROS_INFO("normal: (%f %f %f)", normal.x, normal.y, normal.z);

    ROS_INFO("x: (%f %f %f)", x.x, x.y, x.z);
    ROS_INFO("y: (%f %f %f)", y.x, y.y, y.z);
    ROS_INFO("z: (%f %f %f)", z.x, z.y, z.z);

    
    tf::Matrix3x3 tf_basis(
        x.x, y.x, z.x,
        x.y, y.y, z.y,
        x.z, y.z, z.z);
        
    tf::Vector3 tf_origin(
        current.x,
        current.y,
        current.z);
    
    tf::Pose tf_pose;
    
    tf_pose.setBasis(tf_basis);
    tf_pose.setOrigin(tf_origin);
    tf::poseTFToMsg (tf_pose, pose);
    return true;
  }

  void MeshGlobalPlanner::initialize(const std::string& name, const boost::shared_ptr<tf::TransformListener>& tf_listener_ptr, const std::string& global_frame){

    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle nh_g;
    sub = nh_g.subscribe("mesh_in", 1, &MeshGlobalPlanner::meshCallback, this);
    navigation_mesh_pub = nh_g.advertise<mesh_msgs::TriangleMeshStamped>("navigation_mesh", 1);
	planning_result_pub = nh_g.advertise<mesh_msgs::TriangleMeshStamped>("planning_result_mesh", 1);

    ROS_INFO("Mesh Global Planner initialized.");

    nh.param("tf_timeout", tf_timeout, 1.0);
    
    // optimization parameters
    nh.param("edge_growing_dist", edge_region_growing_max_dist_, 0.1);
    nh.param("edge_growing_angle", edge_region_growing_max_normal_angle_, M_PI/8);
    
    
    // selecting the graph base type
    std::string tmp_graph_base_type;
    nh.param("graph_base_type", tmp_graph_base_type, std::string("face_graph"));
    if(tmp_graph_base_type == "vertex_graph"){
      graph_base_type_ = MeshGlobalPlanner::VertexGraph;
    }else if(tmp_graph_base_type == "face_graph"){
      graph_base_type_ = MeshGlobalPlanner::FaceGraph;
    }else{
      ROS_ERROR("Wrong graph base type used as node parameter, use \"face_graph\" or \"vertex_graph\"!");
      exit(0);
    }

    ROS_INFO("Parameter graph_base_type: %s", tmp_graph_base_type.c_str());
    ROS_INFO("Parameter edge_growing_dist: %f", edge_region_growing_max_dist_);
    ROS_INFO("Parameter edge_growing_angle: %f", edge_region_growing_max_normal_angle_);

    
    this->global_frame = global_frame;
    this->tf_listener_ptr = tf_listener_ptr;
  }

  void MeshGlobalPlanner::meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg){
    ROS_INFO("Received new mesh in the %s frame with %d triangles, %d vertices and %d vertex normals.", 
    mesh_msg->header.frame_id.c_str(), mesh_msg->mesh.vertices.size(), mesh_msg->mesh.triangles.size(), mesh_msg->mesh.vertex_normals.size());
    
    // transform mesh to global frame
    mesh_msgs::TriangleMeshStamped global_mesh;
    bool tf_success = mesh_msgs_transform::transformTriangleMesh(
      global_frame,           // target frame
      mesh_msg->header.stamp, // target time
      *mesh_msg,              // mesh in
      global_frame,           // fixed frame
      global_mesh,            // mesh out
      *tf_listener_ptr        // tf listener
    );
    
    if(!tf_success){
      ROS_ERROR("StampedTriangleMesh transformation to the globalframe failed!");
      return;
    }
    
    // convert mesh to lvr GraphHalfEdgeMesh
    lvr::MeshBufferPtr mesh_buffer_ptr(new lvr::MeshBuffer);
    lvr_ros::fromTriangleMeshToMeshBuffer(global_mesh.mesh, *mesh_buffer_ptr);
    mesh_ptr = lvr::GraphHalfEdgeMesh<VertexType, NormalType>::Ptr(
      new lvr::GraphHalfEdgeMesh<VertexType, NormalType>(mesh_buffer_ptr));
    
    /*
    switch(graph_base_type_){
      case VertexGraph:
		ROS_INFO("vertex graph edge region growing \n max dist: %f \n max normal angle: %f",
		  edge_region_growing_max_dist_,
          edge_region_growing_max_normal_angle_
		);
        mesh_ptr->vertexGraphEdgeRegionGrowing(
          edge_region_growing_max_dist_,
          edge_region_growing_max_normal_angle_
        );
        break;
      case FaceGraph:
        ROS_INFO("face graph edge region growing \n max dist: %f \n max normal angle: %f",
		  edge_region_growing_max_dist_,
          edge_region_growing_max_normal_angle_
		);
        mesh_ptr->faceGraphEdgeRegionGrowing(
          edge_region_growing_max_dist_,
          edge_region_growing_max_normal_angle_
        );
        break;
    }
    */
    
    ROS_INFO("Prepare mesh for navigation...");
   
    //mesh_ptr->prepareGraphForNavigation();
    lvr::GraphHalfEdgeMesh<VertexType, NormalType>::InflationLevel inflation_level;
    inflation_level.LETHAL = 256.0;
    inflation_level.INSCRIBED = 255.0;
    inflation_level.INSCRIBED_RADIUS =  0.10;//0.30;
    inflation_level.INSCRIBED_RADIUS_SQUARED = pow(inflation_level.INSCRIBED_RADIUS, 2);
    inflation_level.MAX_INFLATION_RADIUS = 0.20;// 0.40;
    inflation_level.MAX_INFLATION_RADIUS_SQUARED = pow(inflation_level.MAX_INFLATION_RADIUS, 2);
    inflation_level.ROUGHNESS_THRESHOLD = 0.75 * M_PI;
    inflation_level.HEIGHT_DIFF_THRESHOLD = 0.1;
    mesh_ptr->borderCostInflationVertexGraph(inflation_level);
    //mesh_ptr->vertexGraphCalculateAverageVertexAngles();
    mesh_ptr->vertexGraphCalculateLocalRoughnessAndHeightDifference(0.20);

    ROS_INFO("Finalize mesh and convert it to message...");
    mesh_ptr->finalize();
    
    mesh_msgs::TriangleMeshStamped planning_mesh_msg;
    planning_mesh_msg.header.stamp = global_mesh.header.stamp;
    planning_mesh_msg.header.frame_id = global_mesh.header.frame_id;

    lvr_ros::fromMeshBufferToTriangleMesh(mesh_ptr->meshBuffer(), planning_mesh_msg.mesh);
    
    ROS_INFO("Add navigation colors...");
    mesh_ptr->finalize();
    
    
    float riskiness_factor = 1;
    float roughness_factor = 450;
    float height_diff_factor = 0;//200;
    
    mesh_ptr->vertexGraphCombineVertexCosts(riskiness_factor, roughness_factor, height_diff_factor);
    mesh_ptr->faceGraphCombineVertexCosts(riskiness_factor, roughness_factor, height_diff_factor);
    
    std::vector<float> face_costs, vertex_costs;
    mesh_ptr->getVertexCostsFaceGraph(face_costs);
    mesh_ptr->getVetrexCostsVertexGraph(vertex_costs);
    
    
    
    lvr_ros::intensityToTriangleRainbowColors(face_costs, planning_mesh_msg.mesh, 0, 256);
    lvr_ros::intensityToVertexRainbowColors(vertex_costs, planning_mesh_msg.mesh, 0, 256);

	ROS_INFO("Publish navigation mesh!");
    navigation_mesh_pub.publish(planning_mesh_msg);
  
  }

} /* namespace mesh_navigation */
