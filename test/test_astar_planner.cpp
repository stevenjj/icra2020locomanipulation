#include <stdio.h>
#include <avatar_locomanipulation/walking/footstep_planner.hpp>


using namespace footstep_planner;

int main(int argc, char **argv){
	A_starPlanner planner;

	shared_ptr<Node> test(new Node(0,0));

	// Create Notes
	// std::shared_ptr<Node> begin (new Node(10, 10) );
	// std::shared_ptr<Node> neighbor (new Node(30, 30) );
	// std::shared_ptr<Node> neighbor2 (new Node(50, 50) );

	// Modify Fscore
	// begin->f_score = 500;
	// neighbor->f_score = 5;
	// neighbor2->f_score = 15;

	// OpenSet / Explored Set operations ------------------------------------------------------------------------------------------
	// 1. Create Open Set and add Nodes
	// std::vector< std::shared_ptr<Node> > open_set;
	// std::vector< shared_ptr<Node> >::iterator node_it;

	// open_set.push_back(begin); 
	// open_set.push_back(neighbor); 
	// open_set.push_back(neighbor2); 

	// std::cout << "Open Set Before sorting open_set:" << std::endl;
	// for(int i = 0; i < open_set.size(); i++){
	// 	std::cout << "(val, key) = (" << open_set[i]->f_score << ", " << open_set[i]->key << ")" << std::endl; 
	// }

	// // 2. Sorting the set:
	// NodePtr_Compare_Fcost node_compare_fcost_obj;
	// std::sort(open_set.begin(), open_set.end(), node_compare_fcost_obj);

	// std::cout << "Open Set After sorting open_set:" << std::endl;
	// for(int i = 0; i < open_set.size(); i++){
	// 	std::cout << "(val, key) = (" << open_set[i]->f_score << ", " << open_set[i]->key << ")" << std::endl; 
	// }

	// // 3. Lowest fscore in openset is  the first element:
	// std::cout << "First element: (val, key) = (" << open_set[0]->f_score << ", " << open_set[0]->key << ")" << std::endl; 

	// // 4. Efficient removal of an element (see: stackoverflow.com/questions/30611584)
	// // WARNING: It destroys the ordering of the container.
	// node_it = open_set.begin();
	// *node_it = std::move(open_set.back());
	// open_set.pop_back();

	// std::cout << "Open Set After removing the first element:" << std::endl;
	// for(int i = 0; i < open_set.size(); i++){
	// 	std::cout << "(val, key) = (" << open_set[i]->f_score << ", " << open_set[i]->key << ")" << std::endl; 
	// }

	// // 5. Finding a specific node in the set:
	// std::shared_ptr<Node> node_to_find;
	// node_to_find = neighbor2;

	// node_it = std::find_if(open_set.begin(), open_set.end(), NodePtr_Equality(node_to_find));
	// // If we didn't reach the end of the set, that means we found a node with a matching key.
	// if (node_it != open_set.end()){
	// 	std::cout << "Found the node with key " << (*node_it)->key << " and f_score = " << (*node_it)->f_score << std::endl;
	// }else{
	// 	std::cout << "could not find the node" << std::endl;
	// }

	// // Closed set operations ------------------------------------------------------------------------------------------
	// std::map< std::shared_ptr<Node>, bool, NodePtr_Compare> closed_set;
	// std::map< std::shared_ptr<Node>, bool, NodePtr_Compare>::iterator cs_it;


	// // 1. Inserting elements into the closed_set
	// // std::pair < map::iterator, bool > 
	// std::pair< std::map< std::shared_ptr<Node>, bool, NodePtr_Compare>::iterator, bool> query;
	// query = closed_set.insert( std::pair< std::shared_ptr<Node>, bool >(neighbor, true) );
	// if (query.second == true){
	// 	std::cout << "Successfully inserted node " << std::endl;
	// }else{
	// 	std::cout << "Node is already in the map" << std::endl;
	// }

	// // 2. Check if key is the map
	// if (closed_set.count(neighbor) > 0){
	// 	std::cout << "Found neighbor node in map " <<  "(val, key) = (" << neighbor->f_score << ", " <<  neighbor->key << ")" << std::endl;
	// }else{
	// 	std::cout << "Did not find the neighbor node" << std::endl;
	// }

	// if (closed_set.count(begin) > 0){
	// 	std::cout << "Found Node in map " <<  "(val, key) = (" << begin->f_score << ", " <<  begin->key << ")" << std::endl;
	// }else{
	// 	std::cout << "Did not find the begin node" << std::endl;
	// }

	// // 3. Showing Contents of the map
	// for(cs_it = closed_set.begin(); cs_it != closed_set.end(); cs_it++){
	// 	std::cout << "Node (key, val) = (" << cs_it->first->key << ", " << cs_it->second << ")" << std::endl;
	// }


	// shared_ptr<Node> start (new Node(1,1));
	// shared_ptr<Node> end (new Node(1,1));
	// shared_ptr<Node> neighborx (new Node(5,1));

	// std::cout << "start == end?" << start->key.compare(neighborx->key) << std::endl;
	// std::cout << "start == end?" << start->key.compare(end->key) << std::endl;

	planner.doAstar();
	//planner.getNeighbors(test);

	return 0;
}