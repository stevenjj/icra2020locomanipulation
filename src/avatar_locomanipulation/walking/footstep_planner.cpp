#include <avatar_locomanipulation/walking/footstep_planner.hpp>

#include <map>
#include <string>
#include <utility>
#include <algorithm>
#include <fstream>
#include <ostream>
#include <list>

#define THIS_COM "/home/hcappel1/Documents/"

	void saveVector(const std::vector<double>& _vec, std::string _name, bool b_param = false);
	void saveValue(double _value, std::string _name, bool b_param = false);
	void cleaningFile(std::string file_name_, std::string& ret_file_, bool b_param);
	static std::list<std::string> gs_fileName_string;  // global & static

	void saveVector(const std::vector<double>& _vec, std::string _name, bool b_param) {
    	std::string file_name;
    	cleaningFile(_name, file_name, b_param);
    	std::ofstream savefile(file_name.c_str(), std::ios::app);
    	for (int i(0); i < _vec.size(); ++i) {
        	savefile << _vec[i] << "\t";
    	}
    	savefile << "\n";
    	savefile.flush();
	}

	void cleaningFile(std::string _file_name, std::string& _ret_file, bool b_param) {
    	if (b_param)
        	_ret_file += THIS_COM;
    	else
        	_ret_file += THIS_COM;

    	_ret_file += _file_name;
    	_ret_file += ".txt";

    	std::list<std::string>::iterator iter = std::find(
        	gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    	if (gs_fileName_string.end() == iter) {
        	gs_fileName_string.push_back(_file_name);
        	remove(_ret_file.c_str());
    	}
	}





namespace footstep_planner{

	bool Node_Compare::operator() (const Node & lhs, const Node &rhs) const{
		{
	 		return lhs.key.compare(rhs.key);
	 	}
	}

	bool Node_Compare_Fcost_Greater::operator() (const Node & lhs, const Node &rhs) const{
		{
			return lhs.f_score < rhs.f_score;
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


	// constructor
	NodePtr_Compare_Fcost::NodePtr_Compare_Fcost(){	
	}
	bool NodePtr_Compare_Fcost::operator() (const shared_ptr<Node> & lhs, const shared_ptr<Node> &rhs) const{
		{
			return lhs->f_score < rhs->f_score;
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
		g_score = 100000;
		f_score = 100000;
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
	double A_starPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
		return sqrt(pow(neighbor->x - current->x,2) + pow(neighbor->y - current->y,2));
	}

	// void A_starPlanner::getPath(std::vector<Node> & path){
	// }

	double A_starPlanner::goalDistance(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		return sqrt(pow(goal->x - neighbor->x,2) + pow(goal->y - neighbor->y,2));
	}

	std::vector< shared_ptr<Node> > A_starPlanner::getNeighbors(shared_ptr<Node> & current, int branch, double disc){

		vector<double> x_vals;
		vector<double> y_vals;

		vector< vector<double> > coords;
		vector< shared_ptr<Node> > neighbors;

		double x_new;
		double y_new;

		double temp_val;
		int k = -branch;
		while(k < branch + 1){
			temp_val = disc*k;
			x_new = current->x + temp_val;
			y_new = current->y + temp_val;
			x_vals.push_back(x_new);
			y_vals.push_back(y_new);
			k++;
		}

		for (size_t i(0);i < x_vals.size();i++){
			for (size_t j(0); j < y_vals.size();j++){
				vector<double> x_y_coord;
				x_y_coord.push_back(x_vals[i]);
				x_y_coord.push_back(y_vals[j]);
				saveVector(x_y_coord,"foot coordinates");
				coords.push_back(x_y_coord);
				shared_ptr<Node> neighbor(new Node(x_vals[i],y_vals[j]));
				neighbor->parent = current;
				neighbors.push_back(neighbor);
			}
		}
		cout << "size of set: " << neighbors.size() << endl;
		return neighbors;



	}

	void A_starPlanner::getPath(std::vector< shared_ptr<Node> > & optimal_path, shared_ptr<Node> current_node, shared_ptr<Node> begin){
		while (begin->key.compare(current_node->key) != 0) {
			optimal_path.push_back(current_node);
			current_node = current_node->parent;	
		}
		// Add the parent
		optimal_path.push_back(current_node);
	}

	void A_starPlanner::doAstar(){
		//initial config of the robot
		double x_i = 1;
		double y_i = 1;

		double x_f = 58;
		double y_f = -20;
		//define all maps and queues 
		std::vector< std::shared_ptr<Node> > OpenSet;
		// std::map< std::shared_ptr<Node>, bool, NodePtr_Compare> ClosedSet;
		std::vector< std::shared_ptr<Node> > ClosedSet;
		A_starPlanner a_star;
		shared_ptr<Node> current_node;
		std::vector< shared_ptr<Node> >::iterator node_it_closed;
		std::vector< shared_ptr<Node> >::iterator node_it_open;
		std::vector< shared_ptr<Node> >::iterator node_it;
		//std::map< std::shared_ptr<Node>, bool, NodePtr_Compare>::iterator cs_it;
		//std::pair< std::map< std::shared_ptr<Node>, bool, NodePtr_Compare>::iterator, bool> query;

		// shared_ptr<Node> test (new Node(0,0));
		// test->parent = "test";
		// Explored.push_back(test);





		// std::priority_queue <std::shared_ptr<Node>, std::vector< std::shared_ptr<Node> >, NodePtr_Compare_Fcost_Greater> open_set;
		// std::map< shared_ptr<Node>, shared_ptr<Node>, NodePtr_Compare> cameFrom;
		// std::set<std::shared_ptr<Node>, NodePtr_Compare> ClosedList;
		// std::set<std::shared_ptr<Node>, NodePtr_Compare> explored; 
		// std::shared_ptr<Node> current_node;

		// A_starPlanner a_star;

		//define starting node
		std::shared_ptr<Node> begin (new Node(x_i, y_i) );
		std::shared_ptr<Node> goal(new Node(x_f,y_f) );
		begin->g_score = 0;
		begin->f_score = a_star.goalDistance(begin,goal);
		
		OpenSet.push_back(begin);
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

		// }


		//cout << "size of g_score: " << g_score.size() << endl;
		//cout << "size of f_score: " << f_score.size() << endl;
		// cout << "size of open set: " << openSet.size() << endl;
		while (!OpenSet.empty()){

				//sort the open set
				cout << "size of open set: " << OpenSet.size() << endl;
				NodePtr_Compare_Fcost node_compare_fcost_obj;
				std::sort(OpenSet.begin(), OpenSet.end(), node_compare_fcost_obj);

				
				//choose top value of open set as current node;
				current_node = OpenSet[0];
				cout << current_node->x << "," << current_node->y << endl;
				cout << current_node->f_score << endl;

				//is current node = goal node?
				if (abs(current_node->x - goal->x) < 0.1 && abs(current_node->y - goal->y) < 0.1){
					//reproduce path
					cout << "made it to the end!!!!!" << endl;

					std::vector< shared_ptr<Node> > optimal_path;
					a_star.getPath(optimal_path, current_node, begin);

					std::cout << "got the path I think. It has size:" << optimal_path.size() << std::endl;

					for (size_t i(0); i < optimal_path.size(); i++){
						cout << "optimal path::  " << "x: " << optimal_path[i]->x << "y: " << optimal_path[i]->y << endl;
						vector<double> optimal_coord;
						optimal_coord.push_back(optimal_path[i]->x);
						optimal_coord.push_back(optimal_path[i]->y);
						saveVector(optimal_coord,"optimal_path");
					}

					break;
				}

				//current node not equal to goal
				else{
					//pop current node off the open list
					node_it = OpenSet.begin();
					*node_it = std::move(OpenSet.back());
					OpenSet.pop_back();

					//put current node onto explored set
					ClosedSet.push_back(current_node);
					// ClosedSet.insert( std::pair< std::shared_ptr<Node>, bool >(current_node, true) );

					//create new neighbor nodes
					std::vector< shared_ptr<Node> > neighbors;
					neighbors = a_star.getNeighbors(current_node,3,1);

					//iterate through all neighbors
					for (size_t i(0);i < neighbors.size(); i++){

						std::shared_ptr<Node> node_to_find;
						node_to_find = neighbors[i];
						//check to see if neighbor already in closed set
						node_it_closed = std::find_if(ClosedSet.begin(), ClosedSet.end(), NodePtr_Equality(node_to_find));
						node_it_open = std::find_if(OpenSet.begin(), OpenSet.end(), NodePtr_Equality(node_to_find));

						// If we didn't reach the end of the set, that means we found a node with a matching key.
						if (node_it_closed == ClosedSet.end()){
						//if (ClosedSet.count(neighbors[i]) == 0){	
							//check to see if neighbor already visited

							if (node_it_open != OpenSet.end()){
								double tentative_gscore = current_node->g_score + a_star.gScore(current_node,neighbors[i]) + 2;
								cout << "tentative g_Score" << tentative_gscore << endl;
								if (tentative_gscore < (*node_it_open)->g_score){
									cout << "update" << endl;
									(*node_it_open)->parent = current_node;
									(*node_it_open)->g_score = tentative_gscore;
									(*node_it_open)->f_score = (*node_it_open)->g_score + a_star.goalDistance((*node_it_open),goal); 
			
								}
							}
							else{
								neighbors[i]->g_score = current_node->g_score + a_star.gScore(current_node,neighbors[i]) + 2;
								neighbors[i]->f_score = neighbors[i]->g_score + a_star.goalDistance(neighbors[i],goal);
								OpenSet.push_back(neighbors[i]);
							}		
						}
		
					}
				}
		}
		
	}
}




//update closed set to map
//comment and clean up code
