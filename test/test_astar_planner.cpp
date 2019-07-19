#include <stdio.h>
#include <avatar_locomanipulation/walking/footstep_planner.hpp>

#include <chrono>
#include <ctime>

using namespace planner;






int main(int argc, char **argv){
	A_starPlanner planner;
	XYPlanner xy_planner;

	// shared_ptr<Node> test(std::make_shared<XYNode>(7,7));
	// shared_ptr<Node> test1(std::make_shared<XYNode>(2,2));
	// shared_ptr<Node> test2(std::make_shared<XYNode>(3,3));
	// shared_ptr<Node> test3(std::make_shared<XYNode>(5,0));
	// shared_ptr<Node> test4(std::make_shared<XYNode>(6,0));
	// shared_ptr<Node> test5(std::make_shared<XYNode>(7,0));
	// shared_ptr<Node> test6(std::make_shared<XYNode>(8,0));
	// shared_ptr<Node> test7(std::make_shared<XYNode>(1,1));
	// test->f_score = 2;
	// test1->f_score = 3;
	// test2->f_score = 4;
	// test3->f_score = 1;
	// test4->f_score = 5;
	// test5->f_score = 6;
	// test6->f_score = 10;
	// test7->f_score = 8;

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
	//std::map< std::shared_ptr<Node>, bool>::iterator es_it;


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

	// xy_planner.doAstar();

	shared_ptr<Node> parent_node(std::make_shared<XYNode>(0.0, 1.0));
	// shared_ptr<XYNode> fnode = std::dynamic_pointer_cast< XYNode > (parent_node);
	shared_ptr<XYNode> fnode;
	fnode = std::static_pointer_cast< XYNode > (parent_node);		

	// Create Start Node
	double x_i = 1;
	double y_i = 1;
	shared_ptr<Node> begin (std::make_shared<XYNode>(x_i, y_i));
	// Create Goal Node
	double x_f = 25;
	double y_f = -7;
	shared_ptr<Node> goal (std::make_shared<XYNode>(x_f, y_f));

	// Set Start and End Goals
    xy_planner.setStartNode(begin);
    xy_planner.setGoalNode(goal);

    // Start Timer
    auto start = std::chrono::system_clock::now();
	// Do A Star on the xy 
	xy_planner.doAstar();
	// End Timer
    auto end = std::chrono::system_clock::now();
	//  Print Path and Time
	xy_planner.printPath();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

	// planner.getNeighbors(test,3,1);
	// shared_ptr<Node> test2(new Node(1,0));
	// std::map< std::shared_ptr<Node>, bool, NodePtr_Compare> ClosedSet;
	// ClosedSet.insert( std::pair< std::shared_ptr<Node>, bool >(test, true) );
	// cout << "size of closed set: " << ClosedSet.size() << endl;

	// if (ClosedSet.count(test2) == 0){
	// 	cout << "test2 not in closed set" << endl;

	// }
	// else{
	// 	cout << "test2 in closed set" << endl;
	// }


	// vector< shared_ptr<Node> > OpenSet;

	// OpenSet.push_back(test1);
	// OpenSet.push_back(test2);
	// OpenSet.push_back(test3);
	// OpenSet.push_back(test4);
	// OpenSet.push_back(test5);
	// OpenSet.push_back(test6);
	// OpenSet.push_back(test7);


	// NodePtr_Compare_Fcost node_compare_fcost_obj;
	// // std::cout << "pre sort:" << std::endl;
	// // for (size_t i(0); i < ClosedSet.size();i++){
	// // cout << "Node key: " << ClosedSet[i]->key << endl;
	// // }

	// // std::cout << "test1 compare test2 " << test1->key.compare(test2->key) << std::endl;
	// // std::cout << "test2 compare test3 " << test2->key.compare(test3->key) << std::endl;
	// // std::cout << "test1 compare test3 " << test1->key.compare(test3->key) << std::endl;

	// // NodePtr_Compare_key node_ptr_compare_key_obj;
	// // std::sort(OpenSet.begin(),OpenSet.end(), node_compare_fcost_obj);


	// // int vector_pos;
	// // vector_pos = lower_bound(OpenSet.begin(),OpenSet.end(),test, node_compare_fcost_obj) - OpenSet.begin();
	// // vector< shared_ptr<Node> >::iterator it;
	// // it = OpenSet.begin() + vector_pos;
	// // OpenSet.insert(it,test);

	// // std::cout << "post sort:" << std::endl;
	// // for (size_t i(0); i < OpenSet.size();i++){
	// // 	cout << "Node f_score: " << OpenSet[i]->f_score << endl;
	// // }




	// // int vector_pos;
	// // vector_pos = lower_bound(ClosedSet.begin(),ClosedSet.end(),test,node_ptr_compare_key_obj) - ClosedSet.begin();
	// // cout << "f_scores: " << test7->f_score << endl;
	// // cout << "positon of vector: " << vector_pos << endl;

	// // vector< shared_ptr<Node> >::iterator it;
	// // it = ClosedSet.begin() + vector_pos;
	// // ClosedSet.insert(it,test);

	// // for (size_t i(0); i < ClosedSet.size();i++){
	// // cout << "Node key: " << ClosedSet[i]->key << endl;
	// // }
	// NodePtr_Compare_key node_ptr_compare_key_obj;
	// std::set< std::shared_ptr<Node>, NodePtr_Compare_key> testClosedSet;

	// std::set< shared_ptr<Node> >::iterator node_it;
	// //std::pair<std::set<int>::iterator,bool> ret;


	// //insertion
	// testClosedSet.insert(test);
	// testClosedSet.insert(test1);
	// testClosedSet.insert(test2);
	// testClosedSet.insert(test3);
	// testClosedSet.insert(test4);



	// //searching
	// // it = testClosedSet.find(test1);


	// // if (it != testClosedSet.end()){
	// // 	cout << "found node with key: " << (it*->key) << endl;
	// // }
	// // else{
	// // 	cout << "did not find node" << endl;
	// // }
	// shared_ptr<Node> node_to_find;
	// node_to_find = test1;
	// node_it = std::find_if(testClosedSet.begin(), testClosedSet.end(), NodePtr_Equality(node_to_find));
	// // // If we didn't reach the end of the set, that means we found a node with a matching key.
	// if (node_it != testClosedSet.end()){
	// 	std::cout << "Found the node with key " << (*node_it)->key << " and f_score = " << (*node_it)->f_score << std::endl;
	// }else{
	//  	std::cout << "could not find the node" << std::endl;
	// }

	// // if ((binary_search(ClosedSet.begin(), ClosedSet.end(), test, node_ptr_compare_key_obj))){ 
 // //       cout << "test exists in vector"; 
 // //    }
 // //   	else{
 // //   		cout << "test not in Closed set" << endl;
 // //   	}


	// std::map<shared_ptr<Node>,bool> Exploredtest;

	// Exploredtest.insert(pair<shared_ptr<Node>,bool> (test,true));
	// Exploredtest.insert(pair<shared_ptr<Node>,bool> (test1,true));
	// Exploredtest.insert(pair<shared_ptr<Node>,bool> (test2,true));
	// Exploredtest.insert(pair<shared_ptr<Node>,bool> (test3,true));
	// Exploredtest.insert(pair<shared_ptr<Node>,bool> (test4,true));

	// cout << "size of explored test: " << Exploredtest.size() << endl;
	
	// es_it = Exploredtest.find(test4);
	// if (es_it != Exploredtest.end()){
	// 	cout << "found node with key: " << es_it->first->key << endl;
	// 	cout << "pre change f score of found node: " << es_it->first->f_score << endl;
	// 	es_it->first->f_score = 54321;
	// 	cout << "post change f score of found node: " << es_it->first->f_score << endl;
	// }
	// else{
	// 	cout << "node not present in map" << endl;
	// }


	return 0;
}