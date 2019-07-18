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

	NodePtr_Compare_key::NodePtr_Compare_key(){		
	}
	bool NodePtr_Compare_key::operator() (const shared_ptr<Node> & lhs, const shared_ptr<Node> &rhs) const{
		{
		return (lhs->key.compare(rhs->key) < 0);
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

	shared_ptr<Node> A_starPlanner::Initialization(){
		double x_i = 1;
		double y_i = 1;

		shared_ptr<Node> begin (new Node(x_i, y_i));

		return begin;

	}

	shared_ptr<Node> A_starPlanner::GoalNode(){
		double x_f = 75;
		double y_f = 15;

		shared_ptr<Node> goal (new Node(x_f, y_f));

		return goal;
	}

	// gScore Implementation: computes the cost from going to the current to the neighbor node
	double A_starPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
		return sqrt(pow(neighbor->x - current->x,2) + pow((neighbor->y - current->y),2));
	}

	// heuristicCost Implementation: computes the cost from going from neighbor node to goal node

	double A_starPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		return sqrt(pow(goal->x - neighbor->x,2) + pow(abs(goal->y - neighbor->y),2));
	}

	//Generates neighbors based on current node

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

	//returns the optimal path

	void A_starPlanner::getPath(std::vector< shared_ptr<Node> > & optimal_path, shared_ptr<Node> current_node, shared_ptr<Node> begin){
		while (begin->key.compare(current_node->key) != 0) {
			optimal_path.push_back(current_node);
			current_node = current_node->parent;	
		}
		// Add the parent
		optimal_path.push_back(current_node);
	}

	bool A_starPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
		if (abs(current_node->x - goal->x) < 0.1 && abs(current_node->y - goal->y) < 0.1){
			return true;
		}
		else{
			return false;
		}
	}

	void A_starPlanner::WriteData(vector< shared_ptr<Node> > optimal_path){
		for (size_t i(0); i < optimal_path.size(); i++){
			cout << "optimal path::  " << "x: " << optimal_path[i]->x << "y: " << optimal_path[i]->y << endl;
			vector<double> optimal_coord;
			optimal_coord.push_back(optimal_path[i]->x);
			optimal_coord.push_back(optimal_path[i]->y);
			saveVector(optimal_coord,"optimal_path");
		}
	}

	//perform the A_star algorithm

	bool A_starPlanner::doAstar(){
		A_starPlanner a_star;
		NodePtr_Compare node_ptr_compare_obj;

		NodePtr_Compare_key node_ptr_compare_key_obj;
		NodePtr_Compare_Fcost node_compare_fcost_obj;

		//initial config of the robot
		std::shared_ptr<Node> begin;
		std::shared_ptr<Node> goal;
		begin = a_star.Initialization();
		goal = a_star.GoalNode();

		//Initialize starting node with g score and f score
		begin->g_score = 0;
		begin->f_score = a_star.heuristicCost(begin,goal);

		//define all vectors, maps and iterators 
		shared_ptr<Node> current_node;

		std::vector< std::shared_ptr<Node> > OpenSet; 
		//OpenSet.resize(10000); //preallocate memory
		std::set<shared_ptr<Node>, NodePtr_Compare_key> ClosedSet;
		// ClosedSet.resize(10000); //preallocate memory
		std::set< std::shared_ptr<Node>, NodePtr_Compare_key> ExploredSet;
	
		std::vector< shared_ptr<Node> >::iterator node_it;
		std::set< std::shared_ptr<Node> >::iterator es_it;
		std::set< shared_ptr<Node> >::iterator node_set_it;
		
		OpenSet.push_back(begin); //append starting node to open set



		while (!OpenSet.empty()){

				//sort the open set
				cout << "size of open set: " << OpenSet.size() << endl;
				cout << "size of explored set: " << ExploredSet.size() << endl;
				std::sort(OpenSet.begin(), OpenSet.end(), node_compare_fcost_obj);

				//choose top value of open set as current node;
				current_node = OpenSet[0];

				//is current node = goal node?
				if (a_star.goalReached(current_node, goal) == true){
					//reproduce path
					cout << "made it to the end!!!!!" << endl;

					vector< shared_ptr<Node> > optimal_path;
					a_star.getPath(optimal_path, current_node, begin);

					std::cout << "Optimal path size:" << optimal_path.size() << std::endl;

					a_star.WriteData(optimal_path);
			
					return true;
				}

				//current node not equal to goal
				else{
					//pop current node off the open list
					node_it = OpenSet.begin();
					*node_it = std::move(OpenSet.back());
					OpenSet.pop_back();

					//Erase node from the explored set
					ExploredSet.erase(current_node);

					//insert current node onto closed set
					ClosedSet.insert(current_node); //*****************************

					//create new neighbor nodes
					std::vector< shared_ptr<Node> > neighbors;
					neighbors = a_star.getNeighbors(current_node,3,1);

					//iterate through all neighbors
					for (size_t i(0);i < neighbors.size(); i++){

						//check if node exists in closed set
						node_set_it = ClosedSet.find(neighbors[i]);
						if (node_set_it == ClosedSet.end()){ 
				
							// find neighbor in explored set
							es_it = ExploredSet.find(neighbors[i]);

							//if neighbor exists in explored set
							if (es_it != ExploredSet.end()){ 

								//determine tentative g score
								double tentative_gscore = current_node->g_score + a_star.gScore(current_node,neighbors[i]);
								
								//check to see if g score path is better than previous
								if (tentative_gscore < (*es_it)->g_score){ 

									//update node
									(*es_it)->parent = current_node; 
									(*es_it)->g_score = tentative_gscore; 
									(*es_it)->f_score = tentative_gscore + a_star.heuristicCost(neighbors[i],goal); 
								
								}

							}
							else{
								neighbors[i]->g_score = current_node->g_score + a_star.gScore(current_node,neighbors[i]);
								neighbors[i]->f_score = neighbors[i]->g_score + a_star.heuristicCost(neighbors[i],goal);
								OpenSet.push_back(neighbors[i]);
								ExploredSet.insert(neighbors[i]);
							}		
						}
					}
				}
		}	
	}
}


