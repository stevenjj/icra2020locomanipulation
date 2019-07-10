#include <avatar_locomanipulation/walking/footstep_planner.hpp>

namespace footstep_planner{
	// Node class Implementation ------------------------------
	Node::Node(){
		commonInitialization();
		std::cout << "[Node Constructed]" << std::endl;	
	}
	Node::Node(const double x_in, const double y_in){
		commonInitialization();
		x = x_in;
		y = y_in;
		std::cout << "[Node Constructed] with (x,y) = (" << x << ", " << y << ")"  << std::endl;	
	}

	// Destructor
	Node::~Node(){
		std::cout << "[Node Destroyed]" << std::endl;	
	}

	void Node::commonInitialization(){
		visited = false;
		obstacle = false;
		GlobalGoal = 1000;
		LocalGoal = 1000;
		f_score = GlobalGoal + LocalGoal;
		start = false;
		x = 0.0;
		y = 0.0;
	}


	// A_starPlanner Implementation ------------------------------
	// Constructor
	A_starPlanner::A_starPlanner(){
		std::cout << "[A_starPlanner Constructed]" << std::endl;
	}
	// Destructor
	A_starPlanner::~A_starPlanner(){
		std::cout << "[A_starPlanner Destroyed]" << std::endl;
	}
	// gScore Implementation: computes the cost from going to the current to the neighbor node
	double A_starPlanner::gScore(const Node & current, const Node & neighbor){
		return (neighbor.x - current.x) + (neighbor.y - current.y);
	}

	void A_starPlanner::getPath(std::vector<Node> & path){
		Node goal_node;
		path.push_back(goal_node);
	}

}