/* Copyright (C) 2011 Uni Osnabrück
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
 * GraphHalfEdgeMesh.tcc
 *
 *  @date 25.08.2015
 *  @author Sebastian Pütz (spuetz@uos.de)
 */


namespace lvr
{
	
template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::GraphHalfEdgeMesh( )
  : HalfEdgeMesh<VertexT, NormalT>(), face_cnt(0), vertex_cnt(0)
{
  std::srand(std::time(0));
}

template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::GraphHalfEdgeMesh(
        MeshBufferPtr mesh)
  : HalfEdgeMesh<VertexT, NormalT>(mesh), face_cnt(0), vertex_cnt(0)
{
  std::srand(std::time(0));
}

template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::~GraphHalfEdgeMesh()
{

}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::addVertex(VertexT v)
{
  HalfEdgeMesh<VertexT, NormalT>::addVertex(v);
  boost::add_vertex(vertex_graph);
  vertex_cnt++;
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::addTriangle(uint a, uint b, uint c)
{
  HalfEdgeMesh<VertexT, NormalT>::addTriangle(a, b, c);
  
  std::pair<GraphEdge, bool> edge_insert;
  WeightMap vertex_weightmap = boost::get(boost::edge_weight, vertex_graph);
  WeightMap face_weightmap = boost::get(boost::edge_weight, face_graph);

  edge_insert = boost::add_edge(a, b, vertex_graph);
  if(edge_insert.second){
    vertex_weightmap[edge_insert.first] = this->m_vertices[a]->m_position.distance(this->m_vertices[b]->m_position);
  }
  edge_insert = boost::add_edge(b, c, vertex_graph);
  if(edge_insert.second){
    vertex_weightmap[edge_insert.first] = this->m_vertices[b]->m_position.distance(this->m_vertices[c]->m_position);
  }
  edge_insert = boost::add_edge(c, a, vertex_graph);
  if(edge_insert.second){
    vertex_weightmap[edge_insert.first] = this->m_vertices[c]->m_position.distance(this->m_vertices[a]->m_position);
  }
  
  const int face_index = face_cnt++;
  boost::add_vertex(face_graph);
  typename HalfEdgeFace<VertexT, NormalT>::FaceVector neighbours;
  typename HalfEdgeFace<VertexT, NormalT>::FaceVector::iterator n_iter;
  this->m_faces[face_index]->getAdjacentFaces(neighbours);

  VertexT center = this->m_faces[face_index]->getCentroid();
  for(n_iter = neighbours.begin(); n_iter != neighbours.end(); ++n_iter){
    int neighbour_index = (*n_iter)->m_face_index-1;
    edge_insert = boost::add_edge(face_index, neighbour_index, face_graph);
    if(edge_insert.second){
      face_weightmap[edge_insert.first] = center.distance((*n_iter)->getCentroid());
    }
  }
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphAStar(
  uint start,
  uint goal,
  std::list<int>& path)
{
  if(start < 0 || start >= face_cnt){
    std::cerr << " start vertex id is invalid!" << std::endl;
    return false;
  }

  if(goal < 0 || goal >= face_cnt){
    std::cerr << " goal vertex id is invalid!" << std::endl;
    return false;
  }
  IndexMap indices = boost::get(boost::vertex_index, face_graph);
  path.clear();
  vector<GraphNode> p(boost::num_vertices(face_graph));
  vector<CostType> d(boost::num_vertices(face_graph));
  try{
    //call astar named parameter interface
    boost::astar_search(face_graph,
      start,
      FaceGraphDistanceHeuristic(this->m_faces, indices, goal),  
      boost::predecessor_map(
        boost::make_iterator_property_map(
          p.begin(), boost::get(boost::vertex_index, face_graph)
        )
      ).
      distance_map(
        boost::make_iterator_property_map(
          d.begin(), boost::get(boost::vertex_index, face_graph)
        )
      ).visitor(AStarGoalVisitor(goal))
    );

  } catch(FoundGoal fg) {
    for(GraphNode v = goal;; v = p[v]) {
      path.push_front(v);
      if(p[v] == v)
        break;
    }
    return true;
  }
  return false;
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphAStar(
  uint start,
  uint goal,
  std::list<int>& path)
{
  if(start < 0 || start >= vertex_cnt){
    std::cerr << " start vertex id is invalid!" << std::endl;
    return false;
  }

  if(goal < 0 || goal >= vertex_cnt){
    std::cerr << " goal vertex id is invalid!" << std::endl;
    return false;
  }


  IndexMap indices = boost::get(boost::vertex_index, vertex_graph);
  path.clear();
  vector<GraphNode> p(boost::num_vertices(vertex_graph));
  vector<CostType> d(boost::num_vertices(vertex_graph));
  try{
    //call astar named parameter interface
    boost::astar_search(vertex_graph,
      start,
      VertexGraphDistanceHeuristic(this->m_vertices, indices, goal),  
      boost::predecessor_map(
        boost::make_iterator_property_map(
          p.begin(), boost::get(boost::vertex_index, vertex_graph)
        )
      ).
      distance_map(
        boost::make_iterator_property_map(
          d.begin(), boost::get(boost::vertex_index, vertex_graph)
        )
      ).visitor(AStarGoalVisitor(goal))
    );
  } catch(FoundGoal fg) {
    for(GraphNode v = goal;; v = p[v]) {
      path.push_front(v);
      if(p[v] == v)
        break;
    }
    return true;
  }
  return false;
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphDijkstra(
  uint start,
  uint goal,
  std::list<int>& path)
{
  if(start < 0 || start >= vertex_cnt){
    std::cerr << " start vertex id is invalid!" << std::endl;
    return false;
  }

  if(goal < 0 || goal >= vertex_cnt){
    std::cerr << " goal vertex id is invalid!" << std::endl;
    return false;
  }

  IndexMap indices = boost::get(boost::vertex_index, vertex_graph);
  path.clear();
  vector<GraphNode> p(boost::num_vertices(vertex_graph));
  vector<CostType> d(boost::num_vertices(vertex_graph));
  boost::dijkstra_shortest_paths(vertex_graph, start,
    boost::predecessor_map(boost::make_iterator_property_map(p.begin(),boost::get(boost::vertex_index, vertex_graph))).distance_map(boost::make_iterator_property_map(d.begin(),boost::get(boost::vertex_index, vertex_graph))));
  
  if(p[goal] == goal && goal != start){
    return false;
  }
  for(GraphNode v = goal;; v = p[v]) {
    path.push_front(v);
    if(p[v] == v)
      break;
  }
  return true;
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphDijkstra(
  uint start,
  uint goal,
  std::list<int>& path)
{
  IndexMap indices = boost::get(boost::vertex_index, face_graph);
  path.clear();
  vector<GraphNode> p(boost::num_vertices(face_graph));
  vector<CostType> d(boost::num_vertices(face_graph));
  boost::dijkstra_shortest_paths(face_graph, start,
    boost::predecessor_map(boost::make_iterator_property_map(p.begin(),boost::get(boost::vertex_index, face_graph))).distance_map(boost::make_iterator_property_map(d.begin(),boost::get(boost::vertex_index, face_graph))));

  if(p[goal] == goal && goal != start){
    return false;
  }
  for(GraphNode v = goal;; v = p[v]) {
    path.push_front(v);
    if(p[v] == v)
      break;
  }
  return true;
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getNearestVertexIndex(VertexT vertex, size_t& vertex_index){

  double smallest_dist = std::numeric_limits<double>::max();
  double smallest_index = -1;

  for(size_t i = 0; i < vertex_cnt; i++){
    double dist = vertex.distance(this->m_vertices[i]);
    if( dist < smallest_dist ){
      smallest_dist = dist;
      smallest_index = i;
    }
  } 
  return smallest_index;
}



} // namespace lvr
