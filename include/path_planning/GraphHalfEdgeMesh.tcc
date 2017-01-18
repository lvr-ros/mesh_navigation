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
#include <boost/math/special_functions/fpclassify.hpp>


namespace lvr
{
    
template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::GraphHalfEdgeMesh( )
  : HalfEdgeMesh<VertexT, NormalT>(), face_cnt(0), vertex_cnt(0)
{
}

template<typename VertexT>
inline bool vertexIsValid(VertexT& vertex){
  return
    boost::math::isfinite<float>(vertex.x) &&
    boost::math::isfinite<float>(vertex.y) &&
    boost::math::isfinite<float>(vertex.z);
}

template<typename NormalT>
inline bool normalIsValid(NormalT& normal){
  if(!( boost::math::isfinite<float>(normal.x) &&
        boost::math::isfinite<float>(normal.y) &&
        boost::math::isfinite<float>(normal.z)
  )) return false;
  
  double length = normal.length2();
  return length > 0.99 && length < 1.01;
    
}


template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::GraphHalfEdgeMesh(
        MeshBufferPtr mesh)
  : HalfEdgeMesh<VertexT, NormalT>(), face_cnt(0), vertex_cnt(0)
{
    size_t num_vertices, num_normals, num_faces;
    floatArr vertices = mesh->getVertexArray(num_vertices);
  floatArr normals = mesh->getVertexNormalArray(num_normals);
    // Add all vertices
    const bool insert_normals = num_normals == num_vertices;
    if(!insert_normals){
      std::cerr << "normals are not inserted!" << std::endl;
    }
    
    for(size_t i = 0; i < num_vertices; i++)
    {
      VertexT new_vertex(vertices[3 * i], vertices[3 * i + 1], vertices[3 * i + 2]);
      if(! vertexIsValid(new_vertex)){
        std::cerr << "invalid vertex: " << new_vertex << std::endl;
      }else{
        addVertex(new_vertex);
        if(insert_normals){
          NormalT new_normal(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]);
          if(! normalIsValid(new_normal)){
            std::cerr << "inavlid normal: " << new_normal << std::endl;
            this->addNormal(NormalT(0, 0, 1));
          }else{
            this->addNormal(new_normal);
          }
        }
      }
    }

    // Add all faces
    uintArr faces = mesh->getFaceArray(num_faces);
    for(size_t i = 0; i < num_faces; i++)
    {
      unsigned int a, b, c;
      a = faces[3 * i];
      b = faces[3 * i + 1];
      c = faces[3 * i + 2];
      if( a >= num_vertices || b >= num_vertices || c >= num_vertices){
        std::cerr << " invalid index while triangle insertion: (" << a << ", " << b << ", " << c << ") - max vertex index is: " << num_vertices-1 << std::endl;
      }else{
        addTriangle(a,b,c);
      }
    }

    // Initial remaining stuff
    this->m_globalIndex = 0;
    this->m_regionClassifier = ClassifierFactory<VertexT, NormalT>::get("Default", this);
    this->m_classifierType = "Default";
    this->m_depth = 100;

}

template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::~GraphHalfEdgeMesh()
{

}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::addVertex(VertexT v)
{
  HalfEdgeMesh<VertexT, NormalT>::addVertex(v);
  this->m_vertices[vertex_cnt]->m_index = vertex_cnt;
  GraphNode vertex_insert = boost::add_vertex(vertex_graph);
  // init vertex costs to zero
  VertexCostMap vertex_graph_vertex_costmap = boost::get(vertex_costs_t(), vertex_graph);
  vertex_graph_vertex_costmap[vertex_insert] = 0;
  if(vertex_insert != vertex_cnt){
    std::cerr << "difference in graph and mesh index: " << vertex_insert << ", " << vertex_cnt << std::endl;
  }
  vertex_cnt++;
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::addTriangle(uint a, uint b, uint c)
{
  const int face_index = face_cnt;
  HalfEdgeMesh<VertexT, NormalT>::addTriangle(a, b, c);
  // set face index for the graph relationship
  this->m_faces[face_index]->m_face_index = face_index;
  
  std::pair<GraphEdge, bool> edge_insert;

  EdgeDistanceMap vertex_graph_distances = boost::get(edge_distance_t(), vertex_graph);
  EdgeDistanceMap face_graph_distances = boost::get(edge_distance_t(), face_graph);
  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph);
  
  edge_insert = boost::add_edge(a, b, vertex_graph);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = this->m_vertices[a]->m_position.distance(this->m_vertices[b]->m_position);
  }
  edge_insert = boost::add_edge(b, c, vertex_graph);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = this->m_vertices[b]->m_position.distance(this->m_vertices[c]->m_position);
  }
  edge_insert = boost::add_edge(c, a, vertex_graph);
  if(edge_insert.second){
    vertex_graph_distances[edge_insert.first] = this->m_vertices[c]->m_position.distance(this->m_vertices[a]->m_position);
  }
  
  GraphNode vertex_insert = boost::add_vertex(face_graph);
  // init costs to zero
  face_graph_vertex_costmap[vertex_insert] = 0;
  
  typename HalfEdgeFace<VertexT, NormalT>::FaceVector neighbours;
  typename HalfEdgeFace<VertexT, NormalT>::FaceVector::iterator n_iter;
  this->m_faces[face_index]->getAdjacentFaces(neighbours);

  VertexT center = this->m_faces[face_index]->getCentroid();
  for(n_iter = neighbours.begin(); n_iter != neighbours.end(); ++n_iter){
    int neighbour_index = (*n_iter)->m_face_index;
    edge_insert = boost::add_edge(face_index, neighbour_index, face_graph);
    if(edge_insert.second){
      face_graph_distances[edge_insert.first] = center.distance((*n_iter)->getCentroid());
    }
  }
  
  face_cnt++;
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getVertexCostsFaceGraph(std::vector<float>& costs){
  costs.clear();
  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph);
  for(size_t i=0; i<face_cnt; i++){
    costs.push_back(face_graph_vertex_costmap[i]);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getVetrexCostsVertexGraph(std::vector<float>& costs){
  costs.clear();
  VertexCostMap vertex_graph_vertex_costmap = boost::get(vertex_costs_t(), vertex_graph);
  for(size_t i=0; i<vertex_cnt; i++){
    costs.push_back(vertex_graph_vertex_costmap[i]);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getDistancesFaceGraph(std::vector<float>& costs){
  costs.clear();
  VertexDistanceMap distance_map = boost::get(boost::vertex_distance, face_graph);
  for(size_t i=0; i<face_cnt; i++){
    costs.push_back(distance_map[i]);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getDistancesVertexGraph(std::vector<float>& costs){
  costs.clear();
  VertexDistanceMap distance_map = boost::get(boost::vertex_distance, vertex_graph);
  for(size_t i=0; i<vertex_cnt; i++){
    costs.push_back(distance_map[i]);
  }
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphAStar(
  const VertexT& start,
  const VertexT& goal,
  std::vector<VertexT>& path,
  std::vector<NormalT>& path_normals)
{
    int start_index, goal_index;
    getNearestVertexIndexFaceGraph(start, start_index);
    getNearestVertexIndexFaceGraph(goal, goal_index);

    std::list<int> path_ids;
    if(faceGraphAStar(start_index, goal_index, path_ids)){
      path.clear();
            for(std::list<int>::iterator iter = path_ids.begin();
                iter != path_ids.end(); ++iter)
            {   
                path.push_back(this->m_faces[*iter]->getCentroid());
                path_normals.push_back(this->m_faces[*iter]->getFaceNormal());
            }
            return true;
        }
    else{
            return false;
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
  const VertexT& start,
  const VertexT& goal,
  std::vector<VertexT>& path,
  std::vector<NormalT>& path_normals)
{
    int start_index, goal_index;
    getNearestVertexIndexVertexGraph(start, start_index);
    getNearestVertexIndexVertexGraph(goal, goal_index);

    std::list<int> path_ids;
    if(vertexGraphAStar(start_index, goal_index, path_ids)){
      path.clear();
            for(std::list<int>::iterator iter = path_ids.begin();
                iter != path_ids.end(); ++iter)
            {
                path.push_back(this->m_vertices[*iter]);
                path_normals.push_back(this->m_normals[*iter]);
            }
            return true;
        }
    else{
            return false;
    }
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
  const VertexT& start,
  const VertexT& goal,
  std::vector<VertexT>& path,
  std::vector<NormalT>& path_normals)
{
  int start_index, goal_index;
  getNearestVertexIndexVertexGraph(start, start_index);
  getNearestVertexIndexVertexGraph(goal, goal_index);

  std::list<int> path_ids;
  if(vertexGraphDijkstra(start_index, goal_index, path_ids)){
    path.clear();
    for(std::list<int>::iterator iter = path_ids.begin();
      iter != path_ids.end(); ++iter)
    {
      path.push_back(this->m_vertices[*iter]->m_position);
      path_normals.push_back(this->m_vertices[*iter]->m_normal);
    }
    return true;
  }
  else{
    return false;
  }
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
  PredecessorMap p = boost::get(boost::vertex_predecessor, vertex_graph);
  VertexDistanceMap d = boost::get(boost::vertex_distance, vertex_graph);
  boost::dijkstra_shortest_paths(vertex_graph, start,
    boost::predecessor_map(p).distance_map(d)
  );
  
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
  const VertexT& start,
  const VertexT& goal,
  std::vector<VertexT>& path,
  std::vector<NormalT>& path_normals)
{
    int start_index, goal_index;
    getNearestVertexIndexFaceGraph(start, start_index);
    getNearestVertexIndexFaceGraph(goal, goal_index);

    

    if(start_index == -1 || goal_index == -1)
        return false;

    std::list<int> path_ids;
    if(faceGraphDijkstra(start_index, goal_index, path_ids)){
      path.clear();
        for(std::list<int>::iterator iter = path_ids.begin();
            iter != path_ids.end(); ++iter)
        {
            path.push_back(this->m_faces[*iter]->getCentroid());
            path_normals.push_back(this->m_faces[*iter]->getFaceNormal());
        }
        return true;
    }
    else{
            return false;
    }
}

template<typename VertexT, typename NormalT>
bool GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphDijkstra(
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
  
  PredecessorMap p = boost::get(boost::vertex_predecessor, face_graph);
  VertexDistanceMap d = boost::get(boost::vertex_distance, face_graph);
  
  boost::dijkstra_shortest_paths(face_graph, start,
    boost::predecessor_map(p).distance_map(d)
  );

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
void GraphHalfEdgeMesh<VertexT, NormalT>::getNearestVertexIndexFaceGraph(const VertexT vertex, int& vertex_index){

  double smallest_dist = std::numeric_limits<double>::max();
  int smallest_index = -1;

  for(size_t i = 0; i < face_cnt; i++){
    double dist = vertex.distance(this->m_faces[i]->getCentroid());
    if( dist < smallest_dist && boost::degree(i, face_graph) > 2){
      smallest_dist = dist;
      smallest_index = i;
    }
  } 
  vertex_index = smallest_index;
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::getNearestVertexIndexVertexGraph(const VertexT vertex, int& vertex_index){

  double smallest_dist = std::numeric_limits<double>::max();
  int smallest_index = -1;

  for(size_t i = 0; i < vertex_cnt; i++){
    double dist = vertex.distance(this->m_vertices[i]->m_position);
    if( dist < smallest_dist && boost::degree(i, vertex_graph) > 2 ){
      smallest_dist = dist;
      smallest_index = i;
    }
  } 
  vertex_index = smallest_index;
}


template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::FaceGraphDistanceHeuristic::FaceGraphDistanceHeuristic(
  typename HalfEdgeMesh<VertexT, NormalT>::FaceVector& faces,
  IndexMap& indices, GraphNode goal
) : faces(faces), indices(indices)
{
  goal_vertex = faces[indices[goal]]->getCentroid();  
}

template<typename VertexT, typename NormalT>
CostType GraphHalfEdgeMesh<VertexT, NormalT>::FaceGraphDistanceHeuristic::operator()(
  GraphNode u
)
{
  int index = indices[u];
  VertexT u_vertex = faces[index]->getCentroid();
  return u_vertex.distance(goal_vertex);
}
      
template<typename VertexT, typename NormalT>
GraphHalfEdgeMesh<VertexT, NormalT>::VertexGraphDistanceHeuristic::VertexGraphDistanceHeuristic(
  typename HalfEdgeMesh<VertexT, NormalT>::VertexVector& vertices,
  IndexMap& indices,
  GraphNode goal
) : vertices(vertices), indices(indices)
{
  goal_vertex = vertices[indices[goal]]->m_position;  
}

template<typename VertexT, typename NormalT>
CostType GraphHalfEdgeMesh<VertexT, NormalT>::VertexGraphDistanceHeuristic::operator()(GraphNode u)
{
  int index = indices[u];
  VertexT u_vertex = vertices[index]->m_position;
  return u_vertex.distance(goal_vertex);
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphEdgeRegionGrowing(
    double max_distance,
    double max_normal_angle
){
  double max_normal_angle_cos = cos(max_normal_angle);

  for(size_t i=0; i<vertex_cnt; i++){
    typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector& edges_out
    = this->m_vertices[i]->out;
    typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector::iterator edge_iter;
    
    for(edge_iter = edges_out.begin(); edge_iter != edges_out.end(); ++edge_iter){
      vertexGraphEdgeRegionGrowing(
        i,
        this->m_vertices[i]->m_position,
        this->m_vertices[i]->m_normal,
        (*edge_iter)->end(),
        max_distance,
        max_normal_angle_cos
      );
    }
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphEdgeRegionGrowing(
    double max_distance,
    double max_normal_angle
){
  double max_normal_angle_cos = cos(max_normal_angle);

  for(size_t i=0; i<face_cnt; i++){
    typename HalfEdgeFace<VertexT, NormalT>::FaceVector neighbours;
    typename HalfEdgeFace<VertexT, NormalT>::FaceVector::iterator n_iter;
    this->m_faces[i]->getAdjacentFaces(neighbours);  
    for(n_iter = neighbours.begin(); n_iter != neighbours.end(); ++n_iter){
      faceGraphEdgeRegionGrowing(
        i,
        this->m_faces[i]->getCentroid(),
        this->m_faces[i]->getFaceNormal(),
        *n_iter,
        max_distance,
        max_normal_angle_cos
      );
    }
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphEdgeRegionGrowing(
    const int center_index,
    const VertexT& center_vertex,
    const NormalT& center_normal,
    const typename HalfEdgeMesh<VertexT, NormalT>::VertexPtr current_vertex_ptr,
    double max_distance,
    double max_normal_angle_cos
){  
  typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector& edges_out = current_vertex_ptr->out;
  typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector::iterator edge_iter;
  
  EdgeDistanceMap vertex_graph_distances = boost::get(edge_distance_t(), vertex_graph);

  for(edge_iter = edges_out.begin(); edge_iter != edges_out.end(); ++edge_iter){
    const typename HalfEdgeMesh<VertexT, NormalT>::VertexPtr& vertex_ptr = (*edge_iter)->end();
    int neighbour_index = vertex_ptr->m_index;
    //if there is no edge for the center vertex and the current vertex
    if( neighbour_index != center_index
    && !boost::edge(center_index, neighbour_index, vertex_graph).second
    && fabs(vertex_ptr->m_normal * center_normal) > max_normal_angle_cos
  ){
        double dist = center_vertex.distance(vertex_ptr->m_position);
        if(dist <= max_distance){
            std::pair<GraphEdge, bool> edge_insert =
        boost::add_edge(center_index, neighbour_index, vertex_graph);
            vertex_graph_distances[edge_insert.first] = dist;
            vertexGraphEdgeRegionGrowing(center_index, center_vertex, center_normal, vertex_ptr, max_distance, max_normal_angle_cos);
        }
    }
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphEdgeRegionGrowing(
    const int center_index,
    const VertexT& center_vertex,
    const NormalT& center_normal,
    const typename HalfEdgeMesh<VertexT, NormalT>::FacePtr current_face_ptr,
    double max_distance,
    double max_normal_angle_cos
){  
  typename HalfEdgeFace<VertexT, NormalT>::FaceVector neighbours;
  typename HalfEdgeFace<VertexT, NormalT>::FaceVector::iterator n_iter;
  current_face_ptr->getAdjacentFaces(neighbours);  
  
  EdgeDistanceMap face_graph_distances = boost::get(edge_distance_t(), face_graph);

  for(n_iter = neighbours.begin(); n_iter != neighbours.end(); ++n_iter){
    int neighbour_index = (*n_iter)->m_face_index;
    //if there is no edge for the two vertices
    
    if( neighbour_index != center_index
    && !boost::edge(center_index, neighbour_index, face_graph).second
    && fabs((*n_iter)->getFaceNormal() * center_normal) > max_normal_angle_cos
  ){
        double dist = center_vertex.distance((*n_iter)->getCentroid());
        if(dist <= max_distance){
            std::pair<GraphEdge, bool> edge_insert =
        boost::add_edge(center_index, neighbour_index, face_graph);
            face_graph_distances[edge_insert.first] = dist;
            faceGraphEdgeRegionGrowing(center_index, center_vertex, center_normal, *n_iter, max_distance, max_normal_angle_cos);
        }
    }
  }
}

// input VertexDistanceMap (squared distances to contours)
// output VertexRiskinessMap (inflation level values)
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::borderCostInflationVertexGraph(const InflationLevel& inflation_level){
  std::vector<std::vector<int> > contours;
  findContours(contours);
  
  VertexDistanceMap vertex_distance_map = boost::get(boost::vertex_distance_t(), vertex_graph);
  VertexRiskinessMap vertex_riskiness_map = boost::get(vertex_riskiness_t(), vertex_graph);
  VertexHeightDifferenceMap vertex_height_difference_map = boost::get(vertex_height_difference_t(), vertex_graph); 
  VertexRoughnessMap vertex_roughness_map = boost::get(vertex_roughness_t(), vertex_graph); 

    
  std::vector<int> lethal_by_roughness;
  std::vector<int> lethal_by_height_diff;
  
  for(size_t i=0; i<vertex_cnt; i++){
    vertex_distance_map[i] = std::numeric_limits<CostType>::max();
    if(vertex_height_difference_map[i] >= inflation_level.HEIGHT_DIFF_THRESHOLD){
      lethal_by_height_diff.push_back(i);
    }
    if(vertex_roughness_map[i] >= inflation_level.ROUGHNESS_THRESHOLD){
      lethal_by_roughness.push_back(i);
    }
  }
  
  contours.push_back(lethal_by_roughness);
  contours.push_back(lethal_by_height_diff);

  typename std::vector<std::vector<int> >::iterator con_iter;
  for(con_iter = contours.begin(); con_iter!=contours.end(); ++con_iter){
    if(con_iter->size() > 15)
      borderCostInflationVertexGraph(inflation_level, *con_iter);
  }
  
  for(size_t i=0; i<vertex_cnt; i++){

    //free space
    if(vertex_distance_map[i] > inflation_level.MAX_INFLATION_RADIUS_SQUARED){
      vertex_riskiness_map[i] = 0;
      
    // inflation radius
    }else if(vertex_distance_map[i] > inflation_level.INSCRIBED_RADIUS_SQUARED){
      double alpha = (sqrt(vertex_distance_map[i]) - inflation_level.INSCRIBED_RADIUS) /
		(inflation_level.MAX_INFLATION_RADIUS - inflation_level.INSCRIBED_RADIUS) * M_PI;
      vertex_riskiness_map[i] = inflation_level.INSCRIBED * (cos(alpha)+1)/2.0 ;
      
    // inscribed radius
    }else if(vertex_distance_map[i] > 0){
      vertex_riskiness_map[i] = inflation_level.INSCRIBED;
    
    // lethal
    }else{
      vertex_riskiness_map[i] = inflation_level.LETHAL;
    }
  }
}

// input -> VertexDistanceMap (squared distance to any contour)
// output -> VertexDistanceMap ( squared distance to contour )
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::borderCostInflationVertexGraph(const InflationLevel& inflation_level, std::vector<int>& contour){
  VertexDistanceMap vertex_distance_map = boost::get(boost::vertex_distance_t(), vertex_graph);
  
  std::vector<int>::iterator con_iter;

  for(con_iter = contour.begin(); con_iter != contour.end(); ++con_iter){
	int center_index = *con_iter;
    VertexPtr center_vertex = this->m_vertices[center_index];
    assert(center_vertex->m_index == *con_iter);
    vertex_distance_map[center_index] = 0;
    std::queue<VertexPtr> vertex_queue;
    vertex_queue.push(center_vertex);
    while(!vertex_queue.empty()){
      VertexPtr vertex = vertex_queue.front();
      vertex_queue.pop();
      typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector::iterator tmp_edge_iter;
      for(tmp_edge_iter = vertex->out.begin(); tmp_edge_iter != vertex->out.end(); ++tmp_edge_iter){
        VertexPtr tmp_vertex = (*tmp_edge_iter)->end();
        CostType sqr_dist = center_vertex->m_position.sqrDistance(tmp_vertex->m_position);
        if(sqr_dist > vertex_distance_map[vertex->m_index] 
          && sqr_dist  < vertex_distance_map[tmp_vertex->m_index]){
          vertex_distance_map[tmp_vertex->m_index] = sqr_dist;
          if(sqr_dist <= inflation_level.MAX_INFLATION_RADIUS_SQUARED){
            vertex_queue.push(tmp_vertex);
          }
        }
      }
    }
  }
}

// output -> EdgeAngleMap
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateAngleEdges()
{
  EdgeAngleMap vertex_graph_angles = boost::get(edge_angle_t(), vertex_graph);
  //WeightMap vertex_weightmap = boost::get(boost::edge_weight, vertex_graph);
  //VertexRiskinessMap vertex_riskiness_map = boost::get(vertex_riskiness_t(), vertex_graph);
  
  size_t progress_one = vertex_cnt / 100;
  std::cout << "calculate angle edges" << std::endl;
  for(size_t i=0; i< vertex_cnt; i++){
    if(i%progress_one == 0){
      std::cout << "progress: " << i / progress_one << "%" << std::endl;
    }
    std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, vertex_graph);
    for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
  	  if( vertex_graph_angles[*out_edge_iter.first] == 0 ){
        int target_vertex = boost::target(*out_edge_iter.first, vertex_graph);
        int source_vertex = boost::source(*out_edge_iter.first, vertex_graph);
        
        double edge_angle = acos ( this->m_vertices[source_vertex]->m_normal * this->m_vertices[target_vertex]->m_normal );
        vertex_graph_angles[*out_edge_iter.first] = edge_angle;
      }
    }
  }
}

// input -> EdgeAngleMap
// output -> VertexRoughnessMap
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateAverageVertexAngles()
{
  vertexGraphCalculateAngleEdges();
  std::cout << "calculate average vertex angles" << std::endl;
  
  size_t progress_one = vertex_cnt / 100;

  VertexAverageAngleMap vertex_graph_average_angles = boost::get(vertex_average_angle_t(), vertex_graph);
  EdgeAngleMap vertex_graph_angles = boost::get(edge_angle_t(), vertex_graph);
  for(size_t i=0; i< vertex_cnt; i++){
    if(i%progress_one == 0){
      std::cout << "progress: " << i / progress_one << "%" << std::endl;
    }
    size_t degree = boost::degree(i, vertex_graph);
    if(degree > 0){
      double angle_sum = 0;
      std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, vertex_graph);
      for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
        angle_sum += vertex_graph_angles[*out_edge_iter.first];
      }
      vertex_graph_average_angles[i] = angle_sum / degree;
    }else{
      vertex_graph_average_angles[i] = 0;
    }
  }
}


template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateLocalNeighborhood(VertexPtr reference_vertex, double squared_radius, bool* used_array, std::vector<VertexPtr>& neighborhood)
{
	vertexGraphCalculateLocalNeighborhood(reference_vertex, reference_vertex, squared_radius, used_array, neighborhood);
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateLocalNeighborhood(VertexPtr reference_vertex, VertexPtr current_vertex, double squared_radius, bool* used_array, std::vector<VertexPtr>& neighborhood)
{
	typename vector<EdgePtr>::iterator it;

	for(it = current_vertex->out.begin(); it != current_vertex->out.end(); it++)
	{
		EdgePtr e = *it;
		VertexPtr next = e->end();

		int index = next->m_index;
		bool used = used_array[index];

		if(! used && reference_vertex->m_position.sqrDistance(next->m_position) < squared_radius){
			neighborhood.push_back(next);
			used_array[index] = true;
			vertexGraphCalculateLocalNeighborhood(reference_vertex, next, squared_radius, used_array, neighborhood);
		}
	}
}


// input -> VertexRoughnessMap
// output -> VertexRoughnessMap
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateLocalRoughnessAndHeightDifference(double radius)
{
  vertexGraphCalculateAverageVertexAngles();
  std::cout << "calculated local roughness and local height differences ... " << std::endl;
  double squared_radius = radius * radius;
  
  VertexAverageAngleMap avarage_angles = boost::get(vertex_average_angle_t(), vertex_graph);
  VertexRoughnessMap local_roughness = boost::get(vertex_roughness_t(), vertex_graph);
  VertexHeightDifferenceMap local_height_difference = boost::get(vertex_height_difference_t(), vertex_graph);

  bool* used = new bool[vertex_cnt];
  for(size_t i=0; i<vertex_cnt; i++){
    used[i] = false;
  }
  
  size_t progress_one = vertex_cnt / 100;


  std::vector<VertexPtr> neighborhood;

  for(size_t i=0; i< vertex_cnt; i++){
    if(i%progress_one == 0){
      std::cout << "progress: " << i / progress_one << "%" << std::endl;
    }

    neighborhood.clear();
    VertexPtr vertex = this->m_vertices[i];
	vertexGraphCalculateLocalNeighborhood(vertex, squared_radius, used, neighborhood);
    
    double sum = 0;
    double min = std::numeric_limits<float>::max();
    double max = std::numeric_limits<float>::min();

    typename vector<VertexPtr>::iterator it;

	// reset used falg
	for(it = neighborhood.begin(); it != neighborhood.end(); it++)
	{
      double height = (*it)->m_position.z;
      int index = (*it)->m_index;
      min = std::min(height, min);
      max = std::max(height, max);
      sum += avarage_angles[index];
      used[index] = false;
    }
    local_roughness[i] = sum / neighborhood.size();
    local_height_difference[i] = max - min;
    
  }
  
  delete[] used;
}

// output -> EdgeAngleMap
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphCalculateAngleEdges()
{
  EdgeAngleMap face_graph_angles = boost::get(edge_angle_t(), face_graph);

  for(size_t i=0; i< face_cnt; i++){
    std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, face_graph);
    for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
  	  if( face_graph_angles[*out_edge_iter.first] == 0 ){
        int target_face = boost::target(*out_edge_iter.first, face_graph);
        int source_face = boost::source(*out_edge_iter.first, face_graph);
        
        double edge_angle = acos ( this->m_faces[source_face]->getFaceNormal() * this->m_faces[target_face]->getFaceNormal() );
        face_graph_angles[*out_edge_iter.first] = edge_angle;
      }
    }
  }
}

// input -> EdgeAngleMap
// output -> VertexRoughnessMap
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphCalculateAverageVertexAngles()
{
  faceGraphCalculateAngleEdges();
  
  VertexAverageAngleMap face_graph_average_angles = boost::get(vertex_average_angle_t(), face_graph);
  EdgeAngleMap face_graph_angles = boost::get(edge_angle_t(), face_graph);
  for(size_t i=0; i< face_cnt; i++){
    size_t degree = boost::degree(i, face_graph);
    if(degree > 0){
      double angle_sum = 0;
      std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, face_graph);
      for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
        angle_sum += face_graph_angles[*out_edge_iter.first];
      }
      face_graph_average_angles[i] = angle_sum / degree;
    }else{
      face_graph_average_angles[i] = 0;
    }
  }
}

// input -> VertexRoughnessMap
// output -> VertexRoughnessMap
template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphCalculateLocalRoughnessAndHeightDifference(double radius)
{
  faceGraphCalculateAverageVertexAngles();
  
  double squared_radius = radius * radius;
  VertexAverageAngleMap avarage_angles = boost::get(vertex_average_angle_t(), face_graph);
  VertexRoughnessMap local_roughness = boost::get(vertex_roughness_t(), face_graph);
  VertexHeightDifferenceMap local_height_difference = boost::get(vertex_height_difference_t(), face_graph);

  
  // reset used falgs
  for(size_t i=0; i< face_cnt; i++){
    this->m_faces[i]->m_used = false;
  }

  for(size_t i=0; i< face_cnt; i++){
    size_t degree = boost::degree(i, face_graph);
    if(degree == 0){
      local_roughness[i] = 0;
      local_height_difference[i] = 0;
      continue;
    }
    
    std::queue<int> queue;
    std::vector<int> current_nodes;

    queue.push(i);
    VertexT& vertex = this->m_faces[i]->getCentorid();
    
    while(!queue.empty()){
      int node = queue.front();
      queue.pop();
      
      this->m_faces[node]->m_used = true;
      current_nodes.push_back(node);
      
      std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(node, face_graph);
      for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
        int target = boost::target(*out_edge_iter.first, face_graph);
        if(!this->m_faces[target]->m_used && 
              this->m_faces[target]->getCentroid().sqrDistance(vertex) <= squared_radius){
          queue.push(target);
        }
      }
    }
    double sum = 0;
    double min = std::numeric_limits<float>::max();
    double max = std::numeric_limits<float>::min();
    for(size_t j=0; j<current_nodes.size(); j++){
      int node = current_nodes[j];
      double height = this->m_faces[node]->getCentroid().z;
      min = std::min(height, min);
      max = std::max(height, max);
      sum += avarage_angles[node];
      this->m_faces[node]->m_used = false;
    }
    local_height_difference[i] = max - min;
    local_roughness[i] = sum / current_nodes.size();
  }
}


template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::findContours(std::vector<std::vector<int> >& contours)
{
  // reset the used flags
  for(size_t i=0; i<face_cnt; i++){
    this->m_faces[i]->m_used = false;
    for(size_t k=0; k<3; k++){
      (*(this->m_faces[i]))[k]->used = false;
    }
  }

  // TODO delete the two lines
  //VertexCostMap vertex_costs_face_graph = boost::get(vertex_costs_t(), face_graph);
  //VertexCostMap vertex_costs_vertx_graph = boost::get(vertex_costs_t(), vertex_graph);

  // search for border faces
  for(size_t i = 0; i < face_cnt; i++){
    typename HalfEdgeMesh<VertexT, NormalT>::FacePtr& current_face_ptr = this->m_faces[i];
    
    // if face already used, continue
    if(current_face_ptr->m_used){
      continue;
    }
    
    current_face_ptr->m_used = true;
    
    // access the edges of the current face and check if they have a
    // pair face without a face -> border edge
    for(int k = 0; k < 3; k++){
      
      typename HalfEdgeMesh<VertexT, NormalT>::EdgePtr current
        = (*current_face_ptr)[k];
      
      if(current->used){
        continue;
      }
      current->used = true;
        
      if(!current->hasPair()){
        std::cerr << "no pair edge! " << std::endl;
        continue;
      }
      typename HalfEdgeMesh<VertexT, NormalT>::EdgePtr current_pair
        = current->pair();
      
      if(current_pair->used){
        continue;
      }
      current_pair->used = true;
      
      if(!current_pair->hasFace()){
        // found beginning of a new contour;
        std::vector<int> contour;
        typename HalfEdgeMesh<VertexT, NormalT>::EdgePtr current_edge_in_contour, next;
        typename HalfEdgeMesh<VertexT, NormalT>::VertexPtr current_vertex_in_contour; 
        
        current_edge_in_contour = next = current_pair;
        current_vertex_in_contour = current_edge_in_contour->end();
        contour.push_back(current_vertex_in_contour->m_index);

        //while the contour is not closed
        while( next )
        {
          current_edge_in_contour = next;
          next = 0;
          
          current_vertex_in_contour = current_edge_in_contour->end();
          
          // TODO delete the comment block // just for marking.. and testing
          /*
          if(current_edge_in_contour->hasPair() && current_edge_in_contour->pair()->hasFace()){
            size_t face_index = current_edge_in_contour->pair()->face()->m_face_index;
            vertex_costs_face_graph[face_index] = 1;
          }
          
          size_t vertex_index = current_vertex_in_contour->m_index;
          vertex_costs_vertx_graph[vertex_index] = 1;
          */
          
          typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector& out_edges = current_vertex_in_contour->out;
          typename HalfEdgeMesh<VertexT, NormalT>::EdgeVector::iterator edge_iter;
        
          for(edge_iter = out_edges.begin(); edge_iter != out_edges.end() && !next; ++edge_iter){
            if((*edge_iter)->used){
              continue;
            }
            (*edge_iter)->used = true;
            if(!(*edge_iter)->hasFace()){
              next = *edge_iter;
              contour.push_back(next->end()->m_index);
            }
          }
        }
        contours.push_back(contour);
      }
    }
  }
}


template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateEdgeWeights(float roughness_factor, float height_diff_factor)
{

  VertexCostMap vertex_graph_vertex_costmap = boost::get(vertex_costs_t(), vertex_graph);
  VertexRoughnessMap vertex_graph_vertex_roughness = boost::get(vertex_roughness_t(), vertex_graph);

  EdgeDistanceMap vertex_graph_distances = boost::get(edge_distance_t(), vertex_graph);
  WeightMap vertex_weightmap = boost::get(boost::edge_weight, vertex_graph);
  VertexRiskinessMap vertex_riskiness_map = boost::get(vertex_riskiness_t(), vertex_graph);
  VertexHeightDifferenceMap vertex_graph_height_difference = boost::get(vertex_height_difference_t(), vertex_graph);

  
  for(size_t i=0; i< vertex_cnt; i++){
    std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, vertex_graph);
    for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
  	  int target_vertex = boost::target(*out_edge_iter.first, vertex_graph);
  	  int source_vertex = boost::source(*out_edge_iter.first, vertex_graph);
  	  double edge_risk =  std::max(vertex_riskiness_map[source_vertex], vertex_riskiness_map[target_vertex]);
      double edge_roughness_factor = std::max(vertex_graph_vertex_roughness[source_vertex], vertex_graph_vertex_roughness[target_vertex]) * roughness_factor + 1;
      double edge_height_diff_factor = std::max(vertex_graph_height_difference[source_vertex], vertex_graph_height_difference[target_vertex]) * height_diff_factor + 1;
      vertex_weightmap[*out_edge_iter.first] = vertex_graph_distances[*out_edge_iter.first] * edge_roughness_factor * edge_height_diff_factor + edge_risk;
    }
  }
}


template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphCalculateEdgeWeights(float roughness_factor, float height_diff_factor)
{

  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph);
  VertexRoughnessMap face_graph_vertex_roughness = boost::get(vertex_roughness_t(), face_graph);

  EdgeDistanceMap face_graph_distances = boost::get(edge_distance_t(), face_graph);
  WeightMap face_weightmap = boost::get(boost::edge_weight, face_graph);
  VertexRiskinessMap vertex_riskiness_map = boost::get(vertex_riskiness_t(), face_graph);
  VertexHeightDifferenceMap face_graph_height_difference = boost::get(vertex_height_difference_t(), face_graph);

  
  for(size_t i=0; i< face_cnt; i++){
    std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, face_graph);
    for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
      int target_vertex = boost::target(*out_edge_iter.first, face_graph);
      int source_vertex = boost::source(*out_edge_iter.first, face_graph);
      double edge_risk =  std::max(vertex_riskiness_map[source_vertex], vertex_riskiness_map[target_vertex]);
      double edge_roughness_factor = std::max(face_graph_vertex_roughness[source_vertex], face_graph_vertex_roughness[target_vertex]) * roughness_factor + 1;
      double edge_height_diff_factor = std::max(face_graph_height_difference[source_vertex], face_graph_height_difference[target_vertex]) * height_diff_factor + 1;
      face_weightmap[*out_edge_iter.first] = face_graph_distances[*out_edge_iter.first] * edge_roughness_factor * edge_height_diff_factor + edge_risk;
    }
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphCombineVertexCosts(
	float riskiness_factor,
    float riskiness_norm,
    float roughness_factor,
    float roughness_norm,
    float height_diff_factor,
    float height_diff_norm)
{
  VertexCostMap face_graph_vertex_costmap = boost::get(vertex_costs_t(), face_graph);
  VertexRiskinessMap face_graph_vertex_riskiness = boost::get(vertex_riskiness_t(), face_graph);
  VertexRoughnessMap face_graph_vertex_roughness = boost::get(vertex_roughness_t(), face_graph);
  VertexHeightDifferenceMap face_graph_height_difference = boost::get(vertex_height_difference_t(), face_graph);
  
  riskiness_factor /= riskiness_norm;
  roughness_factor /= roughness_norm;
  height_diff_factor /= height_diff_norm;

  for(size_t i=0; i<face_cnt; i++){
    face_graph_vertex_costmap[i] =
        riskiness_factor * face_graph_vertex_riskiness[i]
      + roughness_factor * face_graph_vertex_roughness[i]
      + height_diff_factor * face_graph_height_difference[i];
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphGetMaxRiskinessRoughnessHeightDiffValues(
    float& riskiness_max,
    float& roughness_max,
    float& hght_diff_max)
{
  VertexRiskinessMap vertex_graph_vertex_riskiness = boost::get(vertex_riskiness_t(), vertex_graph);
  VertexRoughnessMap vertex_graph_vertex_roughness = boost::get(vertex_roughness_t(), vertex_graph);
  VertexHeightDifferenceMap vertex_graph_height_difference = boost::get(vertex_height_difference_t(), vertex_graph);
  
  for(size_t i=0; i<vertex_cnt; i++){
      riskiness_max = std::max(vertex_graph_vertex_riskiness[i], riskiness_max);
      roughness_max = std::max(vertex_graph_vertex_roughness[i], roughness_max);
      hght_diff_max = std::max(vertex_graph_height_difference[i], hght_diff_max);
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphGetMaxRiskinessRoughnessHeightDiffValues(
    float& riskiness_max,
    float& roughness_max,
    float& hght_diff_max)
{
  VertexRiskinessMap face_graph_vertex_riskiness = boost::get(vertex_riskiness_t(), face_graph);
  VertexRoughnessMap face_graph_vertex_roughness = boost::get(vertex_roughness_t(), face_graph);
  VertexHeightDifferenceMap face_graph_height_difference = boost::get(vertex_height_difference_t(), face_graph);
  
  for(size_t i=0; i<face_cnt; i++){
      riskiness_max = std::max(face_graph_vertex_riskiness[i], riskiness_max);
      roughness_max = std::max(face_graph_vertex_roughness[i], roughness_max);
      hght_diff_max = std::max(face_graph_height_difference[i], hght_diff_max);
  }
}


template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCombineVertexCosts(
    float riskiness_factor,
    float riskiness_norm,
    float roughness_factor,
    float roughness_norm,
    float height_diff_factor,
    float height_diff_norm)
{
  VertexCostMap vertex_graph_vertex_costmap = boost::get(vertex_costs_t(), vertex_graph);
  VertexRiskinessMap vertex_graph_vertex_riskiness = boost::get(vertex_riskiness_t(), vertex_graph);
  VertexRoughnessMap vertex_graph_vertex_roughness = boost::get(vertex_roughness_t(), vertex_graph);
  VertexHeightDifferenceMap vertex_graph_height_difference = boost::get(vertex_height_difference_t(), vertex_graph);

  riskiness_factor /= riskiness_norm;
  roughness_factor /= roughness_norm;
  height_diff_factor /= height_diff_norm;

  std::cout <<  " vertex_cnt: " << vertex_cnt << std::endl;
  for(size_t i=0; i<vertex_cnt; i++){
    vertex_graph_vertex_costmap[i] =
        riskiness_factor * vertex_graph_vertex_riskiness[i]
      + roughness_factor * vertex_graph_vertex_roughness[i]
      + height_diff_factor * vertex_graph_height_difference[i];
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::faceGraphCalculateEdgeHeightDifferences()
{
  EdgeHeightDifferenceMap height_difference_map = boost::get(edge_height_difference_t(), face_graph);
  
  for(size_t i=0; i< face_cnt; i++){
    std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, face_graph);
    for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
      
      if(height_difference_map[*out_edge_iter.first] > 0){
        continue;
      }
      int target_vertex = boost::target(*out_edge_iter.first, face_graph);
      int source_vertex = boost::source(*out_edge_iter.first, face_graph);
      VertexT diff = this->m_faces[source]->getCentroid() - this->m_faces[target]->getCentroid();
      height_difference_map[*out_edge_iter.first] = std::abs(diff.z);
    }
  }
}

template<typename VertexT, typename NormalT>
void GraphHalfEdgeMesh<VertexT, NormalT>::vertexGraphCalculateEdgeHeightDifferences()
{
  EdgeHeightDifferenceMap height_difference_map = boost::get(edge_height_difference_t(), vertex_graph);
  
  for(size_t i=0; i< vertex_cnt; i++){
    std::pair<out_edge_iterator, out_edge_iterator> out_edge_iter = boost::out_edges(i, vertex_graph);
    for(; out_edge_iter.first != out_edge_iter.second; ++out_edge_iter.first){
      
      if(height_difference_map[*out_edge_iter.first] > 0){
        continue;
      }
      int target_vertex = boost::target(*out_edge_iter.first, vertex_graph);
      int source_vertex = boost::source(*out_edge_iter.first, vertex_graph);
      VertexT diff = this->m_vertices[source]->m_position - this->m_vertices[target]->m_position;
      height_difference_map[*out_edge_iter.first] = std::abs(diff.z);
    }
  }
}


} // namespace lvr
