#include <stdio.h>
#include <avatar_locomanipulation/planners/a_star_planner.hpp>

#include <chrono>
#include <ctime>

using namespace planner;

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
    std::set<shared_ptr<Node>, NodePtr_Compare_key> ClosedSet;



    OpenSet.push_back(start_node);
    OpenSet.push_back(end_node);

    for(int i = 0; i < OpenSet.size(); i++){
        std::cout << OpenSet[i]->key << std::endl;
    }



	return 0;
}


