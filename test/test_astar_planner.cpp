#include <stdio.h>
#include <avatar_locomanipulation/walking/footstep_planner.hpp>

#include <chrono>
#include <ctime>

using namespace planner;

int main(int argc, char **argv){
	A_starPlanner planner;
	// XYPlanner xy_planner;

	// // Create Start Node
	// shared_ptr<Node> begin (std::make_shared<XYNode>(1.0, 1.0));
	// // Create Goal Node
	// shared_ptr<Node> goal (std::make_shared<XYNode>(63.0, -7.0));
	// // Set Start and End Goals
 //    xy_planner.setStartNode(begin);
 //    xy_planner.setGoalNode(goal);

 //    // Start Timer
 //    auto start = std::chrono::system_clock::now();
	// // Do A Star on the xy 
	// xy_planner.doAstar();
	// // End Timer
 //    auto end = std::chrono::system_clock::now();
	// //  Print Path and Time
	// xy_planner.printPath();
 //    std::chrono::duration<double> elapsed_seconds = end-start;
 //    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";


    FootstepPlanner footstepplanner;

    // std::shared_ptr<FootstepNode> current_test;
    // std::shared_ptr<Node> current_test_node;
    // shared_ptr<FootstepNode> neighbor_test;
    // shared_ptr<FootstepNode> goal_test;


    //shared_ptr<Node> current (std::make_shared<FootstepNode>(-4.0,5.0,1.0,1.0,0.0,0.0,"RF"));
    // current_test = std::static_pointer_cast<FootstepNode>(current);
    // current_test->turn = true;
    // current_test_node = static_pointer_cast<Node>(current_test);
    // neighbor_test = static_pointer_cast<FootstepNode>(current_test_node);
    // cout << "neighbor test turn: " << neighbor_test->turn << endl;
    // shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(1.0,1.0,1.0,-1.0,0.0,0.0));
    // neighbor_test = std::static_pointer_cast<FootstepNode>(neighbor);

    // shared_ptr<Node> goal_node (std::make_shared<FootstepNode>(1.0,1.0,1.1,-1.1,0.0,0.0));
    // goal_test = std::static_pointer_cast<FootstepNode>(goal_node);

    // current_test->turn = true;

    // //cout << "g score: " << footstepplanner.gScore(current_test,neighbor_test) << endl;
    // cout << "heuristic cost: " << footstepplanner.heuristicCost(neighbor_test,goal_test) << endl;

    // cout << "goal reached? " << footstepplanner.goalReached(neighbor_test,goal_test) << endl;

    // cout << "node has parameters: " << " xLF: " << test1->xLF << " yLF: " << test1->yLF << " xRF: " << test1->xRF << " yRF: " << test1->yRF << " thetaLF: " << test1->thetaLF << " thetaRF: " << test1->thetaRF << endl;
    // cout << "node has key: " << test1->key << endl;

    //footstepplanner.getNeighbors(current);

    shared_ptr<Node> begin_footstep (std::make_shared<FootstepNode>(1.0,1.0,1.0,-1.0,0.0,0.0,"LF"));
    shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(10.0,11.0,10.0,9.0,0.0,-M_PI/4,"RF"));

    footstepplanner.setStartNode(begin_footstep);
    footstepplanner.setGoalNode(goal_footstep);

    footstepplanner.doAstar();

    footstepplanner.printPath();
    footstepplanner.WriteData();


	return 0;
}