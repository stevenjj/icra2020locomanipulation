#include <avatar_locomanipulation/walking/footstep_planner.hpp>

namespace footstep_planner{

	bool Node_Compare::operator() (const Node & lhs, const Node &rhs) const{
		{
	 		return lhs.key.compare(rhs.key);
	 	}
	}

	bool Node_Compare_Fcost_Greater::operator() (const Node & lhs, const Node &rhs) const{
		{
			return lhs.f_score > rhs.f_score;
		}
	}
 

	bool NodePtr_Compare::operator() (const shared_ptr<Node> & lhs, const shared_ptr<Node> &rhs) const{
		{
	 		return lhs->key.compare(rhs->key);
	 	}
	}

	NodePtr_Equality::NodePtr_Equality(shared_ptr<Node> current_node_in){
		current_node_ = current_node_in;
	}
	bool NodePtr_Equality::operator() (const shared_ptr<Node> & nodeObj) const {
		return (current_node_->key.compare(nodeObj->key) == 0);
	}


	bool NodePtr_Compare_Fcost_Greater::operator() (const shared_ptr<Node> & lhs, const shared_ptr<Node> &rhs) const{
		{
			return lhs->f_score > rhs->f_score;
		}
	}


	// Node class Implementation ------------------------------
	Node::Node(){
		commonInitialization();
		std::cout << "[Node Constructed]" << std::endl;	
	}
	Node::Node(const double x_in,const double y_in){
		commonInitialization();
		x = x_in;
		y = y_in;
		key = (to_string(x) + "-" + to_string(y));	
	}

	// Destructor
	Node::~Node(){
		// std::cout << "[Node Destroyed] with key " << this->key << std::endl;	
	}

	void Node::commonInitialization(){
		x = 0.0;
		y = 0.0;
		g_score = 1000;
		f_score = 1000;
		key = (to_string(x) + "-" + to_string(y));		
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

	// void A_starPlanner::getPath(std::vector<Node> & path){
	// }

	double A_starPlanner::goalDistance(const Node neighbor, const Node goal){
		return (goal.x - neighbor.x) + (goal.y - neighbor.y);
	}

	std::vector<Node> A_starPlanner::getNeighbors(Node & current){

		vector<double> x_vals;
		vector<double> y_vals;

		vector< vector<double> > coords;
		vector<Node> neighbors;

		double x_new;
		double y_new;

		double temp_val;
		int k = -1;
		while(k < 2){
			temp_val = 1*k;
			x_new = current.x + temp_val;
			y_new = current.y + temp_val;
			x_vals.push_back(x_new);
			y_vals.push_back(y_new);
			k++;
		}

		for (size_t i(0);i < x_vals.size();i++){
			for (size_t j(0); j < y_vals.size();j++){
				vector<double> x_y_coord;
				x_y_coord.push_back(x_vals[i]);
				x_y_coord.push_back(y_vals[j]);
				coords.push_back(x_y_coord);
				Node neighbor(x_vals[i],y_vals[j]);
				neighbors.push_back(neighbor);
			}
		}
		cout << "size of set: " << neighbors.size() << endl;
		return neighbors;



	}

	void A_starPlanner::doAstar(){
		//initial config of the robot
		double x_i = 1;
		double y_i = 1;

		double x_f = 2;
		double y_f = 2;
		//define all maps and queues 
		std::priority_queue <Node, std::vector<Node>, Node_Compare_Fcost_Greater> openSet;
		std::map<Node, double> fScore; 
		// std::set<Node> evaluated_nodes;

		std::vector<Node> closedSet;
		std::map<Node, Node, Node_Compare> cameFrom;
		Node current_node;

		// A_starPlanner a_star;

		// //define starting node
		// Node begin(x_i,y_i);
		// Node goal(x_f,y_f);
		// begin.g_score = 0;
		// begin.f_score = a_star.goalDistance(begin,goal);
		
		// openSet.push(begin);
		// vector<Node> InitNeighbors;
		// A_starPlanner a_star;
		// InitNeighbors = a_star.getNeighbors(begin);
		// for (size_t i(0);i < InitNeighbors.size();i++){
		// 	string key_val = InitNeighbors[i].key;
		// 	bool exist = false;
		// 	for(int j=0; j < closedSet.size();j++){
		// 		if (closedSet[j].key == key_val){
		// 			exist = true;
		// 		}
		// 	}
		// 	if (exist == false){
		// 		openSet.push(InitNeighbors[i]);
		// 		cameFrom.insert(pair<Node,Node>(InitNeighbors[i],begin));
		// 		InitNeighbors[i].g_score = a_star.gScore(begin,InitNeighbors[i]);
		// 		//g_score.insert(pair<Node, double>(InitNeighbors[i],InitNeighbors[i].g_score));
		// 		InitNeighbors[i].f_score = InitNeighbors[i].g_score + a_star.goalDistance(InitNeighbors[i],goal);
		// 	}
			

		// }


		//cout << "size of g_score: " << g_score.size() << endl;
		//cout << "size of f_score: " << f_score.size() << endl;
		// cout << "size of open set: " << openSet.size() << endl;

	// while (!openSet.empty()){
	// 	current_node = openSet.front();
	// 	if (abs(current_node.x - goal.x) < 0.1 && abs(current_node.y - goal.y) < 0.1){
	// 		//reproduce path
	// 	}

	// 	else {
	// 		openSet.pop();
	// 		closedSet.push_back(current_node);
	// 		vector<Node> Neighbors;
	// 		Neighbors = a_star.getNeighbors(current_node);
	// 		for (size_t n(0);n < Neighbors.size();n++){
	// 			string key_val = Neighbors[n].key;
	// 			bool exist = false;
	// 			for(int j=0; j < closedSet.size();j++){
	// 				if (closedSet[j].key == key_val){
	// 				exist = true;
	// 				}
	// 			}
	// 			if (exist == false){

				
				
	// 			}
	// 			tentative_gscore = current_node.g_score + a_star.gScore(current_node,Neighbors[n]);

	// 			if (tentative_gscore < Neighbors[n].g_score){
	// 				cameFrom[Neighbors[n]] = current_node;
	// 				InitNeighbors[n].g_score = tentative_gscore;
	// 				InitNeighbors[n].f_score = InitNeighbors[n].g_score + a_star.goalDistance(InitNeighbors[n],goal);
	// 				if 
	// 			}
	// 		}
	// 	} 

	}
}











		/*

	    // The set of discovered nodes that need to be (re-)expanded.
	    // Initially, only the start node is known.
	    openSet := {start}

	    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
	    cameFrom := an empty map

	    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
	    gScore := map with default value of Infinity
	    gScore[start] := 0

	    // For node n, fScore[n] := gScore[n] + h(n).
	    fScore := map with default value of Infinity
	    fScore[start] := h(start)

	    while openSet is not empty
	        current := the node in openSet having the lowest fScore[] value
	        if current = goal
	            return reconstruct_path(cameFrom, current)

	        openSet.Remove(current)

	        for each neighbor of current

	            // d(current,neighbor) is the weight of the edge from current to neighbor
	            // tentative_gScore is the distance from start to the neighbor through current
	            tentative_gScore := gScore[current] + d(current, neighbor)

	            if tentative_gScore < gScore[neighbor]
	            // This path to neighbor is better than any previous one. Record it!
	                cameFrom[neighbor] := current
	                gScore[neighbor] := tentative_gScore
	                fScore[neighbor] := gScore[neighbor] + h(neighbor)
	                if neighbor not in openSet
	                    openSet.Add(neighbor)
	    return failure
		*/


