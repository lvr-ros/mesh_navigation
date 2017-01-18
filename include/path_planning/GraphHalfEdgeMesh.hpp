/* Copyright (C) 2015 Uni Osnabrück
 * This file is part of the LAS VEGAS Reconstruction Toolkit,
 *
 * LAS VEGAS is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * LAS VEGAS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 */


/*
 * GraphHalfEdgeMesh.hpp
 *
 *  @date 25.08.2015
 *  @author Sebastian Pütz (spuetz@uos.de)
 */

#ifndef GRAPHHALFEDGEMESH_H_
#define GRAPHHALFEDGEMESH_H_

#include <lvr/geometry/HalfEdgeMesh.hpp>

#include <vector>
#include <list>
#include <math.h>
#include <cstdlib>
#include <limits>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/properties.hpp>

typedef float cost;

// Vertex Costs
struct vertex_costs_t {
  typedef boost::vertex_property_tag kind;
};

// Edge distances - Euclidean vertex distances 
struct edge_distance_t {
  typedef boost::edge_property_tag kind;
};

// Edge Angles
struct edge_angle_t {
  typedef boost::edge_property_tag kind;
};

// Height Differences between nodes
struct edge_height_difference_t {
  typedef boost::edge_property_tag kind;
};

// Roughness - Rough Terrain
struct vertex_roughness_t {
  typedef boost::vertex_property_tag kind;	
};

// Riskiness - Border Vertices etc.
struct vertex_riskiness_t {
  typedef boost::vertex_property_tag kind;
};
// Average Angles
struct vertex_average_angle_t {
  typedef boost::vertex_property_tag kind;
};
// Local Height Difference
struct vertex_height_difference_t {
  typedef boost::vertex_property_tag kind;
};


typedef boost::adjacency_list_traits<boost::listS, boost::vecS, boost::undirectedS>::vertex_descriptor vertex_descriptor;

typedef boost::adjacency_list<
  boost::listS,
  boost::vecS,
  boost::undirectedS, 
  boost::property<boost::vertex_distance_t, cost,
  boost::property<boost::vertex_predecessor_t, vertex_descriptor,
  boost::property<vertex_costs_t, cost,
  boost::property<vertex_riskiness_t, cost,
  boost::property<vertex_roughness_t, cost,
  boost::property<vertex_average_angle_t, cost,
  boost::property<vertex_height_difference_t, cost> > > > > > >,
  boost::property<boost::edge_weight_t, cost,
  boost::property<edge_angle_t, cost,
  boost::property<edge_distance_t, cost,
  boost::property<edge_height_difference_t, cost > > > >
> Graph;

typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
typedef std::pair<out_edge_iterator, out_edge_iterator> out_edge_iterator_range;
typedef boost::graph_traits < Graph >::adjacency_iterator adjacency_iterator;


typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
typedef boost::property_map<Graph, vertex_costs_t>::type VertexCostMap;
typedef boost::property_map<Graph, boost::vertex_distance_t>::type VertexDistanceMap;
typedef boost::property_map<Graph, boost::vertex_predecessor_t>::type PredecessorMap;
typedef boost::property_map<Graph, vertex_riskiness_t>::type VertexRiskinessMap;
typedef boost::property_map<Graph, vertex_roughness_t>::type VertexRoughnessMap;
typedef boost::property_map<Graph, vertex_average_angle_t>::type VertexAverageAngleMap;
typedef boost::property_map<Graph, vertex_height_difference_t>::type VertexHeightDifferenceMap;


typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;
typedef boost::property_map<Graph, edge_distance_t>::type EdgeDistanceMap;
typedef boost::property_map<Graph, edge_angle_t>::type EdgeAngleMap;
typedef boost::property_map<Graph, edge_height_difference_t>::type EdgeHeightDifferenceMap;


typedef Graph::vertex_descriptor GraphNode;
typedef Graph::edge_descriptor GraphEdge;
typedef Graph::vertex_iterator GraphNode_iterator;
typedef float CostType;

namespace lvr
{


/**
 * @brief A implementation of a half edge triangle mesh.
 */
template<typename VertexT, typename NormalT>
class GraphHalfEdgeMesh : public HalfEdgeMesh<VertexT, NormalT>
{
public:

  typedef boost::shared_ptr< ::lvr::GraphHalfEdgeMesh<VertexT, NormalT> > Ptr;

  typedef HalfEdge< HalfEdgeVertex<VertexT, NormalT>, HalfEdgeFace<VertexT, NormalT> > HEdge;
  typedef HalfEdgeFace<VertexT, NormalT> HFace;
  typedef HalfEdgeVertex<VertexT, NormalT> HVertex;


  typedef HEdge* EdgePtr;
  typedef HFace* FacePtr;
  typedef HVertex* VertexPtr;
  
  GraphHalfEdgeMesh();

  /**
   * @brief   Creates a HalfEdgeMesh from the given mesh buffer
   */
  GraphHalfEdgeMesh( MeshBufferPtr model);

  /**
   * @brief   Dtor.
   */
  virtual ~GraphHalfEdgeMesh();
  
  struct InflationLevel{
	CostType LETHAL;
	CostType INSCRIBED;
	CostType INSCRIBED_RADIUS;
	CostType INSCRIBED_RADIUS_SQUARED;
	CostType MAX_INFLATION_RADIUS;
	CostType MAX_INFLATION_RADIUS_SQUARED;
	CostType ROUGHNESS_THRESHOLD;
	CostType HEIGHT_DIFF_THRESHOLD;
  };

  /*

   * @brief   This method should be called every time
   *      a new vertex is created.
   *
   * @param v     A supported vertex type. All used vertex types
   *          must support []-access.
   */
  virtual void addVertex(VertexT v); 

  /** 
   * @brief   Insert a new triangle into the mesh
   *
   * @param a   The first vertex of the triangle
   * @param b   The second vertex of the triangle
   * @param c   The third vertex of the triangle
   */
  virtual void addTriangle(uint a, uint b, uint c);

  void vertexGraphCalculateEdgeWeights(float roughness_factor, float height_diff_factor);

  void faceGraphCalculateEdgeWeights(float roughness_factor, float height_diff_factor);

  bool vertexGraphAStar(uint start, uint goal, std::list<int>& path);
  
  bool vertexGraphAStar(const VertexT& start, const VertexT& goal, std::vector<VertexT>& path, std::vector<NormalT>& path_normals);
  
  bool faceGraphAStar(uint start, uint goal, std::list<int>& path);
  
  bool faceGraphAStar(const VertexT& start, const VertexT& goal, std::vector<VertexT>& path, std::vector<NormalT>& path_normals);

  bool vertexGraphDijkstra(uint start, uint goal, std::list<int>& path);
  
  bool vertexGraphDijkstra(const VertexT& start, const VertexT& goal, std::vector<VertexT>& path, std::vector<NormalT>& path_normals);
  
  bool faceGraphDijkstra(uint start, uint goal, std::list<int>& path);
  
  bool faceGraphDijkstra(const VertexT& start, const VertexT& goal, std::vector<VertexT>& path, std::vector<NormalT>& path_normals);
  
  void borderCostInflationVertexGraph(const InflationLevel& inflation_level);
  
  void borderCostInflationVertexGraph(const InflationLevel& inflation_level, std::vector<int>& contour);
  
  void vertexGraphCalculateAngleEdges();
  
  void faceGraphCalculateAngleEdges();
  
  void faceGraphCalculateAverageVertexAngles();
  
  void vertexGraphCalculateAverageVertexAngles();

  void vertexGraphCalculateLocalNeighborhood(VertexPtr reference_vertex, double squared_radius, bool* used_array, std::vector<VertexPtr>& neighborhood);

  void vertexGraphCalculateLocalNeighborhood(VertexPtr reference_vertex, VertexPtr current_vertex, double squared_radius, bool* used_array, std::vector<VertexPtr>& neighborhood);

  void faceGraphCalculateLocalRoughnessAndHeightDifference(double radius);
  
  void vertexGraphCalculateLocalRoughnessAndHeightDifference(double radius);
  
  void faceGraphCalculateEdgeHeightDifferences();
  
  void vertexGraphCalculateEdgeHeightDifferences();
 
  void vertexGraphEdgeRegionGrowing(
    double max_distance,
    double max_normal_angle
  );
  
  void vertexGraphEdgeRegionGrowing(
    const int center_index,
    const VertexT& center_vertex,
    const NormalT& center_normal,
    const typename HalfEdgeMesh<VertexT, NormalT>::VertexPtr current_vertex_ptr,
    double max_distance,
    double max_normal_angle_cos
  );
  
  void faceGraphEdgeRegionGrowing(
    double max_distance,
    double max_normal_angle
  );
  
  void faceGraphEdgeRegionGrowing(
    const int center_index,
    const VertexT& center_vertex,
    const NormalT& center_normal,
    const typename HalfEdgeMesh<VertexT, NormalT>::FacePtr current_face_ptr,
    double max_distance,
    double max_normal_angle_cos
  );
  
  uint getRandomVertexID(){
    int random = std::rand();
    return (uint)(this->m_vertices.size()*(float)random / RAND_MAX)-1;
  }

  uint getRandomFaceID(){
    int random = std::rand();
    return (uint)(this->m_faces.size()*(float)random / RAND_MAX)-1;
  }

  typename HalfEdgeMesh<VertexT, NormalT>::VertexVector& getVertices(){
    return this->m_vertices;
  }
  typename HalfEdgeMesh<VertexT, NormalT>::FaceVector& getFaces(){
    return this->m_faces;
  }

  void getNearestVertexIndexFaceGraph(const VertexT vertex, int& vertex_index);
  
  void getNearestVertexIndexVertexGraph(const VertexT vertex, int& vertex_index);

  void findContours(std::vector<std::vector<int> >& contours);
  
  void getVertexCostsFaceGraph(std::vector<float>& costs);
  
  void getVetrexCostsVertexGraph(std::vector<float>& costs);
  
  void getDistancesFaceGraph(std::vector<float>& costs);
  
  void getDistancesVertexGraph(std::vector<float>& costs);

  void vertexGraphGetMaxRiskinessRoughnessHeightDiffValues(
    float& riskiness_max,
    float& roughness_max,
    float& height_diff_max
  );
  
  void vertexGraphCombineVertexCosts(
  	float riskiness_factor,
    float riskiness_norm,
    float roughness_factor,
    float roughness_norm,
    float height_diff_factor,
    float height_diff_norm
  );

  void faceGraphGetMaxRiskinessRoughnessHeightDiffValues(
    float& riskiness_max,
    float& roughness_max,
    float& height_diff_max
  );

  void faceGraphCombineVertexCosts(
  	float riskiness_factor,
    float riskiness_norm,
    float roughness_factor,
    float roughness_norm,
    float height_diff_factor,
    float height_diff_norm
  );

private:

  Graph face_graph, vertex_graph;
  size_t face_cnt;
  size_t vertex_cnt;
  struct FoundGoal{};  

  class AStarGoalVisitor : public boost::default_astar_visitor
  {
    public:
      AStarGoalVisitor(GraphNode goal) : goal(goal){}

      template<class GraphT>
      void examine_vertex(GraphNode u, GraphT& g){
        if(u == goal)
          throw FoundGoal();
      }
    private:
      GraphNode goal;
  };

  class FaceGraphDistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
  {
    public:
      FaceGraphDistanceHeuristic(
        typename HalfEdgeMesh<VertexT, NormalT>::FaceVector& faces,
        IndexMap& indices,
        GraphNode goal
      );
      CostType operator()(GraphNode u);
    private:
      VertexT goal_vertex;
      IndexMap indices;
      typename HalfEdgeMesh<VertexT, NormalT>::FaceVector& faces;
  };
  
  class VertexGraphDistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
  {
    public:
      VertexGraphDistanceHeuristic(
        typename HalfEdgeMesh<VertexT, NormalT>::VertexVector& vertices,
        IndexMap& indices,
        GraphNode goal
      );
      CostType operator()(GraphNode u);
    private:
      VertexT goal_vertex;
      IndexMap indices;
      typename GraphHalfEdgeMesh<VertexT, NormalT>::VertexVector vertices;
  };
};

} // namespace lvr


#include "GraphHalfEdgeMesh.tcc"

#endif /* GRAPHHALFEDGEMESH_H_ */
