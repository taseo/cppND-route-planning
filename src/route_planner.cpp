#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // converts inputs to percentage
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // finds closest nodes to start and end coordinates
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node *neighbor_node : current_node->neighbors) {
        neighbor_node->parent = current_node;
        neighbor_node->h_value = this->CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value + current_node->distance(*neighbor_node);
        neighbor_node->visited = true;

        this->open_list.push_back(neighbor_node);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sorts in DESC by h_value + g_value
    std::sort(this->open_list.begin(), this->open_list.end(), CompareNodes);

    RouteModel::Node* nextNode = this->open_list.back();

    this->open_list.pop_back();

    return nextNode;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    this->distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);

    while(1) {
        this->distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;

        path_found.push_back(*current_node);
       
        if (current_node == this->start_node) {
            break;
        }
    }

    // vector needs to be in correct order with start node as the first element
    std::reverse(path_found.begin(), path_found.end()); 

    distance *= m_Model.MetricScale(); // multiply the distance by the scale of the map to get meters

    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;

    // mark start node as visited to not re-visit it again
    current_node->visited = true;

    this->AddNeighbors(current_node);

    while(this->open_list.size()) {
        if (current_node == this->end_node) {
            m_Model.path = this->ConstructFinalPath(current_node);
            break;
        }

        current_node = this->NextNode();
        this->AddNeighbors(current_node);
    }
}