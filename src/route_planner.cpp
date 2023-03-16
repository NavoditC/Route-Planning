#include "route_planner.h"
#include <algorithm>
using std::sort;
using std::reverse;
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x,start_y);
  	this->end_node = &m_Model.FindClosestNode(end_x,end_y);
}


//CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*(this->end_node));
}


//AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for(RouteModel::Node* neighbor : current_node->neighbors)
  {
    neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
    neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
    neighbor->parent = current_node;
    this->open_list.push_back(neighbor); 
    neighbor->visited = true;
  }
}


// NextNode method to sort the open list and return the next node.
bool RoutePlanner::compare(RouteModel::Node *a, RouteModel::Node *b)
{
  return ((a->g_value+a->h_value)<(b->g_value+b->h_value));
}

RouteModel::Node *RoutePlanner::NextNode() {
  sort(open_list.begin(), open_list.end(), RoutePlanner::compare);
  RouteModel::Node* curr = open_list[0];
  open_list.erase(open_list.begin());
  return curr;
}

// ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node!=start_node)
    {
      path_found.push_back(*current_node);
      distance += current_node->distance(*(current_node->parent));
      current_node = current_node->parent;
    }
    path_found.push_back(*start_node);
    reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// A* Search algorithm 
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    this->start_node->visited=true;
    open_list.push_back(current_node);
    
    // TODO: Implement your solution here.
    while(open_list.size()>0)
    {
      current_node = RoutePlanner::NextNode();
      if(current_node == this->end_node)
      {
        m_Model.path = RoutePlanner::ConstructFinalPath(end_node);
        return;
      }
      RoutePlanner::AddNeighbors(current_node);
    }
}