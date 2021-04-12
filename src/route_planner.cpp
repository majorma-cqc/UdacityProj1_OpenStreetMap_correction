//
//  route_planner.cpp
//  real_proj1
//
//  Created by major ma on 2021/3/29.
//

#include "route_planner.h"
#include <algorithm>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    
    //。问题：重复命名start_node和end_node了
    //RouteModel::Node start_node = m_Model.FindClosestNode(start_x, start_y);
    //RouteModel::Node end_node = m_Model.FindClosestNode(end_x, end_y);
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return (node -> distance(*end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node -> FindNeighbors();
    for (RouteModel::Node *d_node : (*current_node).neighbors){
        //  Mistake hier
        d_node -> g_value = current_node -> g_value + d_node -> distance(*current_node);
        d_node -> h_value = CalculateHValue(d_node);
        d_node -> parent = current_node;
        d_node -> visited = true;
        open_list.push_back(d_node);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(RouteModel::Node *node_g, RouteModel::Node *node_s){
    return ((node_g -> h_value + node_g -> g_value) > (node_s -> h_value + node_s -> g_value));
}


void CellSort(std::vector<RouteModel::Node* > *o_list_ptr){
    sort(o_list_ptr -> begin(), o_list_ptr -> end(), Compare);
}



RouteModel::Node *RoutePlanner::NextNode() {
    CellSort(&open_list);
    RouteModel::Node* ptr_2_s = open_list[open_list.size() - 1];
    open_list.pop_back();
    return ptr_2_s;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    
    //  问题：对distance和g_value的定义理解有问题
    distance = current_node -> g_value;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    //while ((current_node -> parent) != start_node)
    //问题：现在的点非空
    while (current_node -> parent != nullptr){
        path_found.push_back(*current_node);
        //std::reverse(path_found.begin(), path_found.end());
        //上面这句可以写在外面，一起排个序
        
        //  在把*current_node放进final_path中之后，更新distance
        distance += current_node -> distance(*(current_node -> parent));
        current_node = (current_node -> parent);
        //float dis = current_node -> distance(*(current_node -> parent));
        //distance += dis;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;
    while (current_node != end_node) {
        //  AddNeighbors(current_node);
        //  问题: 顺序不对
        current_node = NextNode();
        AddNeighbors(current_node);
    }
    m_Model.path = ConstructFinalPath(current_node);
}
