#include <stdio.h>
#include <avatar_locomanipulation/planners/a_star_planner.hpp>

#include <chrono>
#include <ctime>

using namespace planner;

/*
void addNodesTodeleteSet(nodeList){
    deletedSet.clear();
    for node in nodelist:
        add node to deletedSet
}

bool path_to_start_exists(vertex, candidates){
    // Clear candidate list and assign the vertex to the current
    candidates.clear();
    current = vertex;

    // Assume validNodes always have the start node
    while(true){
        if (current->isStartNode){
            return true;
        }
    
        // We have to check the node's ancestors
        else{
            candidates.push_back(current);
            if (current in validNodes){
                // This node has ancestors which leads back to the start node
                return true;
            }else if (current in invalidNodes){
                // This node has an ancestor which is part of the deletedSet
                return false;
            }else if (current in deletedSet){
                // The node is part of the deletedSet
                return false;
            }else{  
                // This is not in the deleted set. Potentially valid, so let us look at the ancestor
                current = current->parent;
                // Continue checking if path to the start node exists
                continue;
            }
        }

    }


    return false;
}

void checkSetValidity(Set, set_type){
    std::vector< std::shared_ptr<Node> > candidates;

    for vertex in Set:
       // Assign whether the vertex is in the open set or closed set
        vertex_assignment[vertex] = set_type;
        if path_to_start_exists(vertex, candidates){
            validNodes.push_back(elem for elem in candidates);
        }else{
            inValidNodes.push_back(elem for elem in candidates);
        }

}

void filterValidNodes(){
    // member variables
    set<vertex> validNodes;
    set<vertex> invalidNodes;
    map<vertex, int> vertex_assignment; // open or closed set 

    validNodes.clear();
    invalidNodes.clear();
    vertex_assignment.clear();

    // Initially assign the start node to be a valid node
    validNodes.push_back(begin);

    // Check the validity of the nodes in each set
    checkSetValidity(ExploredSet, open_set_member);
    checkSetValidity(ClosedSet, open_set_member);

    // Clear the contents of the open and closed sets
    OpenSet.clear();
    ExploredSet.clear();
    ClosedSet.clear();

    for vertex in validNodes:
        if (vertex_assignment[vertex] == open_set_member){
            OpenSet.push_back(vertex);
            ExploredSet.insert(vertex);
        }else if (vertex_assignment[vertex] == closed_set_member){
            ClosedSet.insert(vertex);
        }
}
*/

int main(int argc, char **argv){
	A_starPlanner planner;

    std::cout << "hello world" << std::endl;
    double x_start = 0.0; double y_start = 0.0;
    shared_ptr<Node> start_node (std::make_shared<XYNode>(x_start, y_start));

    double x_end = 10.0; double y_end = 10.0;
    shared_ptr<Node> end_node (std::make_shared<XYNode>(x_end, y_end));

    XYPlanner xy_planner;

    xy_planner.setStartNode(start_node);
    xy_planner.setGoalNode(end_node);

    xy_planner.doAstar();
    xy_planner.printPath();

    NodePtr_Compare_key node_ptr_compare_key_obj;
    NodePtr_Compare_Fcost node_compare_fcost_obj;

    std::vector< std::shared_ptr<Node> > OpenSet; 
    std::set< std::shared_ptr<Node>, NodePtr_Compare_key> ExploredSet;
    std::set<shared_ptr<Node>, NodePtr_Compare_key> ClosedSet;


    OpenSet.push_back(start_node);
    OpenSet.push_back(end_node);

    for(int i = 0; i < OpenSet.size(); i++){
        std::cout << OpenSet[i]->key << std::endl;
    }



	return 0;
}


