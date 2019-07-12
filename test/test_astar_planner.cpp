#include <stdio.h>
#include <avatar_locomanipulation/walking/footstep_planner.hpp>


using namespace footstep_planner;

int main(int argc, char **argv){
	A_starPlanner planner;

	// Node node();
	// Node current(0,0);

	// planner.getNeighbors(current);



	// planner.doAstar();

	// Node begin(10,10);
	// Node neighbor(70,25);
	// Node neighbor2(140,0);

	// std::shared_ptr<Node> begin;
	// begin = std::shared_ptr<Node> (new  Node(10, 10) );
	std::shared_ptr<Node> begin (new Node(10, 10) );
	std::shared_ptr<Node> neighbor (new Node(30, 30) );
	std::shared_ptr<Node> neighbor2 (new Node(50, 50) );


	std::priority_queue <std::shared_ptr<Node>, std::vector< std::shared_ptr<Node> >, NodePtr_Compare_Fcost_Greater> open_set;
	// std::set< std::shared_ptr<Node> > explored;
	// std::set< std::shared_ptr<Node> > closedSet;

	begin->f_score = 500;
	neighbor->f_score = 5;
	neighbor2->f_score = 15;

	open_set.push(begin);
	open_set.push(neighbor);
	open_set.push(neighbor2);

	std::shared_ptr<Node> smallest_cost_node;
	while(!open_set.empty()){
		smallest_cost_node = open_set.top();
		std::cout << "min key: " << smallest_cost_node->key << " fscore:" << smallest_cost_node->f_score << std::endl;
		open_set.pop();
	}

	std::set<std::shared_ptr<Node>, NodePtr_Compare> explored;
	explored.insert(begin); 
	explored.insert(neighbor); 

	std::shared_ptr<Node> node_to_find;
	node_to_find = begin;

	std::set< shared_ptr<Node> >::iterator node_it;

	node_it = std::find_if(explored.begin(), explored.end(), NodePtr_Equality(node_to_find));

	if (node_it != explored.end()){
		std::cout << "Found the node with key " << (*node_it)->key << " and f_score = " << (*node_it)->f_score << std::endl;
	}

	// std::string stringA;
	// std::string stringB;
	// std::string stringC;	
	// stringA = "0.01-0.02";
	// stringB = "0.04-0.04";
	// stringC = "0.80-0.04";	

	// std::map<Node, Node, Node_Compare> cameFrom;
	// std::map<Node, double> gscore;
	// cameFrom[neighbor2] = neighbor;
	// cameFrom[neighbor] = current;

	// std::cout << "neighbor2 came from" << cameFrom[neighbor2].key << std::endl;


	// std::vector<Node>::iterator node_it;

	// Node smallest_cost_node;
	// while(!open_set.empty()){
	// 	smallest_cost_node = open_set.top();
	// 	std::cout << "min key: " << smallest_cost_node.key << " gscore:" << smallest_cost_node.g_score << std::endl;
	// 	open_set.pop();
	// }

	// std::cout << "------------------------- Program Ends --------" << std::endl;

	// // std::cout << "a vs b = " << stringA.compare(stringB) << std::endl;
	// // std::cout << "b vs c = " << stringB.compare(stringC) << std::endl;
	// // std::cout << "a vs a = " << stringA.compare(stringA) << std::endl;

	// // std::cout << "Gscore between current and neighbor = " << planner.gScore(current, neighbor) << std::endl;

	return 0;
}