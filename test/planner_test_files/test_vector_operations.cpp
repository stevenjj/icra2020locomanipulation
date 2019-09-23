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

    // NodePtr_Compare_key node_ptr_compare_key_obj;
    // NodePtr_Compare_Fcost node_compare_fcost_obj;

    // std::vector< std::shared_ptr<Node> > OpenSet; 
    // std::set< std::shared_ptr<Node>, NodePtr_Compare_key> ExploredSet;
    // std::set<shared_ptr<Node>, NodePtr_Compare_key> ClosedSet;


    // OpenSet.push_back(start_node);
    // OpenSet.push_back(end_node);

    xy_planner.OpenSet.clear();
    xy_planner.ExploredSet.clear();
    xy_planner.ClosedSet.clear();

    //std::static_pointer_cast<Node>(node)
    shared_ptr<Node> v_start (std::make_shared<XYNode>(0.0, 0.0));
    shared_ptr<Node> v1 (std::make_shared<XYNode>(1.0, 0.0));
    shared_ptr<Node> v2 (std::make_shared<XYNode>(2.0, 0.0));
    shared_ptr<Node> v3 (std::make_shared<XYNode>(3.0, 0.0));
    shared_ptr<Node> v4 (std::make_shared<XYNode>(4.0, 0.0));
    shared_ptr<Node> v5 (std::make_shared<XYNode>(5.0, 0.0));
    shared_ptr<Node> v6 (std::make_shared<XYNode>(6.0, 0.0));
    shared_ptr<Node> v_goal (std::make_shared<XYNode>(7.0, 0.0));
    shared_ptr<Node> v8 (std::make_shared<XYNode>(8.0, 0.0));
    shared_ptr<Node> v9 (std::make_shared<XYNode>(9.0, 0.0));
    shared_ptr<Node> v10 (std::make_shared<XYNode>(10.0, 0.0));
    shared_ptr<Node> v11 (std::make_shared<XYNode>(11.0, 0.0));
    shared_ptr<Node> v12 (std::make_shared<XYNode>(12.0, 0.0));

    // define edges
    v1->parent = v_start; 
    v2->parent = v1; 
    v3->parent = v1;
    v4->parent = v3;
    v5->parent = v3;
    v6->parent = v3; 
    v_goal->parent = v6;

    v8->parent = v_start;
    v9->parent = v8;
    v10->parent = v8;
    v11->parent = v9;
    v12->parent = v9;

    xy_planner.setStartNode(v_start);
    xy_planner.setGoalNode(v_goal);

    xy_planner.OpenSet.push_back(v2); xy_planner.ExploredSet.insert(v2);
    xy_planner.OpenSet.push_back(v4); xy_planner.ExploredSet.insert(v4);
    xy_planner.OpenSet.push_back(v5); xy_planner.ExploredSet.insert(v5);
    xy_planner.OpenSet.push_back(v10); xy_planner.ExploredSet.insert(v10);
    xy_planner.OpenSet.push_back(v11); xy_planner.ExploredSet.insert(v11);
    xy_planner.OpenSet.push_back(v12); xy_planner.ExploredSet.insert(v12);

    xy_planner.ClosedSet.insert(v_start);
    xy_planner.ClosedSet.insert(v1);
    xy_planner.ClosedSet.insert(v3);
    xy_planner.ClosedSet.insert(v6);
    xy_planner.ClosedSet.insert(v8);
    xy_planner.ClosedSet.insert(v9);
    xy_planner.ClosedSet.insert(v_goal);

    std::cout << "Open Set" << std::endl;
    for(int i = 0; i < xy_planner.OpenSet.size(); i++){
        std::cout << xy_planner.OpenSet[i]->key << std::endl;
    }

    std::cout << "Closed Set" << std::endl;
    for (std::set< std::shared_ptr<Node> >::iterator it = xy_planner.ClosedSet.begin(); it!=xy_planner.ClosedSet.end(); ++it){
        std::cout << (*it)->key << std::endl;
    }

    xy_planner.filterValidNodes();

    std::cout << "Post Filter" << std::endl;
    std::cout << "Open Set" << std::endl;
    for(int i = 0; i < xy_planner.OpenSet.size(); i++){
        std::cout << xy_planner.OpenSet[i]->key << std::endl;
    }

    std::cout << "Closed Set" << std::endl;
    for (std::set< std::shared_ptr<Node> >::iterator it = xy_planner.ClosedSet.begin(); it!=xy_planner.ClosedSet.end(); ++it){
        std::cout << (*it)->key << std::endl;
    }

    // std::vector< std::shared_ptr<Node> > candidates;
    // xy_planner.checkPathToStartExists(xy_planner.OpenSet[0], candidates);


	return 0;
}


