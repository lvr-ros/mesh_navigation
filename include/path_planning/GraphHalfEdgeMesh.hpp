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

// specify some types
typedef float cost;

typedef boost::adjacency_list<
  boost::listS,
  boost::vecS,
  boost::undirectedS,
  boost::no_property,
  boost::property<boost::edge_weight_t, cost> > Graph;

typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;
typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

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

	GraphHalfEdgeMesh();

	/**
	 * @brief   Creates a HalfEdgeMesh from the given mesh buffer
	 */
	GraphHalfEdgeMesh( MeshBufferPtr model);

	/**
	 * @brief   Dtor.
	 */
	virtual ~GraphHalfEdgeMesh();

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

  bool vertexGraphAStar(uint start, uint goal, std::list<int>& path);
  
  bool faceGraphAStar(uint start, uint goal, std::list<int>& path);

  bool vertexGraphDijkstra(uint start, uint goal, std::list<int>& path);
  
  bool faceGraphDijkstra(uint start, uint goal, std::list<int>& path);

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

private:
  Graph face_graph, vertex_graph;
  size_t face_cnt;
  size_t vertex_cnt;
  struct FoundGoal{};
  
  void getNearestVertexIndex(VertexT vertex, size_t& vertex_index);

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
      FaceGraphDistanceHeuristic(typename HalfEdgeMesh<VertexT, NormalT>::FaceVector& faces, IndexMap& indices, GraphNode goal)
        : faces(faces), indices(indices)
      {
        goal_vertex = faces[indices[goal]]->getCentroid();  
      }
      CostType operator()(GraphNode u)
      {
        int index = indices[u];
        VertexT u_vertex = faces[index]->getCentroid();
        return u_vertex.distance(goal_vertex);
      }
    private:
      VertexT goal_vertex;
      IndexMap indices;
      typename HalfEdgeMesh<VertexT, NormalT>::FaceVector& faces;
  };
  
  class VertexGraphDistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
  {
    public:
      VertexGraphDistanceHeuristic(typename HalfEdgeMesh<VertexT, NormalT>::VertexVector& vertices, IndexMap& indices, GraphNode goal)
        : vertices(vertices), indices(indices)
      {
        goal_vertex = vertices[indices[goal]]->m_position;  
      }
      CostType operator()(GraphNode u)
      {
        int index = indices[u];
        VertexT u_vertex = vertices[index]->m_position;
        return u_vertex.distance(goal_vertex);
      }
    private:
      VertexT goal_vertex;
      IndexMap indices;
      typename GraphHalfEdgeMesh<VertexT, NormalT>::VertexVector vertices;
  };

};

} // namespace lvr


#include "GraphHalfEdgeMesh.tcc"

#endif /* GRAPHHALFEDGEMESH_H_ */
