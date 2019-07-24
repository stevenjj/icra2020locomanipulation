#include <avatar_locomanipulation/walking/footstep_planner.hpp>

#include <map>
#include <string>
#include <utility>
#include <algorithm>
#include <fstream>
#include <ostream>
#include <list>

#define THIS_COM "/home/hcappel1/Documents/"
FILE * pFileTXT;

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





namespace planner{

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
		// std::cout << "[Node Constructed]" << std::endl;	
	}

	// Destructor
	Node::~Node(){
		// std::cout << "[Node Destroyed] with key " << this->key << std::endl;	
	}

	XYNode::XYNode(){		
		commonInitialization();
	}
	XYNode::~XYNode(){		
	}

	XYNode::XYNode(const double x_in,const double y_in){
		commonInitialization();
		x = x_in;
		y = y_in;
		key = (to_string(x) + "-" + to_string(y));	
	}

	void XYNode::commonInitialization(){
		x = 0.0;
		y = 0.0;
		g_score = 100000;
		f_score = 100000;
		key = (to_string(x) + "-" + to_string(y));	
	}

	XYPlanner::XYPlanner(){	
		std::cout << "[XYPlanner Constructed]" << std::endl;
	}
	// Destructor
	XYPlanner::~XYPlanner(){
		std::cout << "[XYPlanner Destroyed]" << std::endl;
	}

	double XYPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
		current_ = std::static_pointer_cast<XYNode>(current);
		neighbor_ = std::static_pointer_cast<XYNode>(neighbor);
		return sqrt(pow(neighbor_->x - current_->x,2) + pow((neighbor_->y - current_->y),2));
	}

	double XYPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		goal_ = std::static_pointer_cast<XYNode>(goal);
		neighbor_ = std::static_pointer_cast<XYNode>(neighbor);
		return sqrt(pow(goal_->x - neighbor_->x,2) + pow(abs(goal_->y - neighbor_->y),2));
	}

	bool XYPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
		current_ = std::static_pointer_cast<XYNode>(current_node);
		goal_ = std::static_pointer_cast<XYNode>(goal);
		if (abs(current_->x - goal_->x) < 0.1 && abs(current_->y - goal_->y) < 0.1){
			return true;
		}
		else{
			return false;
		}
	}


	void XYPlanner::WriteData(vector< shared_ptr<Node> > optimal_path){
		for (size_t i(0); i < optimal_path.size(); i++){
			opt_node_ = std::static_pointer_cast<XYNode>(optimal_path[i]);
			cout << "optimal path::  " << "x: " << opt_node_->x << "y: " << opt_node_->y << endl;
			vector<double> optimal_coord;
			optimal_coord.push_back(opt_node_->x);
			optimal_coord.push_back(opt_node_->y);
			saveVector(optimal_coord,"optimal_path");
		}
	}

	void XYPlanner::printPath(){
		for (size_t i(0); i < optimal_path.size(); i++){
			opt_node_ = std::static_pointer_cast<XYNode>(optimal_path[i]);
			cout << "optimal path::  " << "x: " << opt_node_->x << "y: " << opt_node_->y << endl;
		}		
		std::cout << "Optimal path size:" << optimal_path.size() << std::endl;
	}

	std::vector< shared_ptr<Node> > XYPlanner::getNeighbors(shared_ptr<Node> & current){
		cout << "get neighbors running..." << endl;
		current_ = std::static_pointer_cast<XYNode>(current);

		int branch = 3;
		double disc = 1.0;

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
			x_new = current_->x + temp_val;
			y_new = current_->y + temp_val;
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
				shared_ptr<Node> neighbor(std::make_shared<XYNode>(x_vals[i],y_vals[j]));
				neighbor->parent = current_;
				neighbors.push_back(neighbor);
			}
		}
		// cout << "size of set: " << neighbors.size() << endl;
		return neighbors;
	}



	//**********************************************************************************
	//**********************************************************************************
	//**********************************************************************************


	FootstepNode::FootstepNode(){		
	commonInitializationFootstep();
	}
	FootstepNode::~FootstepNode(){		
	}

	FootstepNode::FootstepNode(const double xLF_in,const double yLF_in,const double xRF_in,const double yRF_in,const double thetaLF_in, const double thetaRF_in, string turn_in){
		commonInitializationFootstep();
		xLF = xLF_in;
		yLF = yLF_in;
		xRF = xRF_in;
		yRF = yRF_in;

		thetaLF = thetaLF_in;
		thetaRF = thetaRF_in;

		turn = turn_in;

		key = (to_string(xLF) + "_" + to_string(yLF) + "_" + to_string(xRF) + "_" + to_string(yRF) + "_" + to_string(thetaLF) + "_" + to_string(thetaRF) + "_" + turn);	
	}

	void FootstepNode::commonInitializationFootstep(){
		xLF = 0.0;
		yLF = 0.0;
		xRF = 0.0;
		yRF = 0.0;

		thetaLF = 0.0;
		thetaRF = 0.0;

		g_score = 100000;
		f_score = 100000;


		key = (to_string(xLF) + "_" + to_string(yLF) + "_" + to_string(xRF) + "_" + to_string(yRF) + "_" + to_string(thetaLF) + "_" + to_string(thetaRF) + "_" + turn);	
	}

	FootstepPlanner::FootstepPlanner(){	
		std::cout << "[FootstepPlanner Constructed]" << std::endl;
	}
	// Destructor
	FootstepPlanner::~FootstepPlanner(){
		std::cout << "[FootstepPlanner Destroyed]" << std::endl;
	}

	double FootstepPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
		current_ = std::static_pointer_cast<FootstepNode>(current);
		neighbor_ = std::static_pointer_cast<FootstepNode>(neighbor);


		double midpt_curr_x;
		double midpt_curr_y;
		double midpt_neigh_x;
		double midpt_neigh_y;
		double distance;

		midpt_curr_x = (current_->xRF + current_->xLF)/2;
		midpt_curr_y = (current_->yRF + current_->yLF)/2;

		midpt_neigh_x = (neighbor_->xRF + neighbor_->xLF)/2;
		midpt_neigh_y = (neighbor_->yRF + neighbor_->yLF)/2;

		
		//distance = sqrt(pow(neighbor_->xLF - goal_->xLF,2) + pow(neighbor_->yLF - goal_->yLF,2));
		// if (current_->turn == "LF"){
		// 	distance = sqrt(pow(current_->xRF - neighbor_->xLF,2) + pow(current_->yRF - neighbor_->yLF,2));
		// }
		// else if (current_->turn == "RF"){
		// 	distance = sqrt(pow(current_->xLF - neighbor_->xRF,2) + pow(current_->yLF - neighbor_->yRF,2));
		// }
		distance = sqrt(pow(midpt_curr_x - midpt_neigh_x,2) + pow(midpt_curr_y - midpt_neigh_y,2));
		return distance;
	}

	double FootstepPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		goal_ = std::static_pointer_cast<FootstepNode>(goal);
		neighbor_ = std::static_pointer_cast<FootstepNode>(neighbor);

		double midpt_neigh_x;
		double midpt_neigh_y;
		double midpt_goal_x;
		double midpt_goal_y;
		double distance;

		double midpt_neigh_theta;
		double midpt_goal_theta;
		// double distance1;
		// double distance2;

		midpt_neigh_x = (neighbor_->xRF + neighbor_->xLF)/2;
		midpt_neigh_y = (neighbor_->yRF + neighbor_->yLF)/2;

		midpt_goal_x = (goal_->xRF + goal_->xLF)/2;
		midpt_goal_y = (goal_->yRF + goal_->yLF)/2;

		midpt_neigh_theta = (neighbor_->thetaLF + neighbor_->thetaRF)/2;
		midpt_goal_theta = (goal_->thetaLF + goal_->thetaRF)/2;

		double theta_err = abs(midpt_goal_theta - midpt_neigh_theta);

		
		//distance = sqrt(pow(neighbor_->xLF - goal_->xLF,2) + pow(neighbor_->yLF - goal_->yLF,2));
		// // }
		// // else{
		// distance2 = sqrt(pow(neighbor_->xRF - goal_->xRF,2) + pow(neighbor_->yRF - goal_->yRF,2));

		distance = sqrt(pow(midpt_neigh_x - midpt_goal_x,2) + pow(midpt_neigh_y - midpt_goal_y,2)) + theta_err;
		// }

		return distance;

	}

	bool FootstepPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
		current_ = std::static_pointer_cast<FootstepNode>(current_node);
		goal_ = std::static_pointer_cast<FootstepNode>(goal);

		double midpt_curr_x;
		double midpt_curr_y;
		double midpt_goal_x;
		double midpt_goal_y;
		double distance;
		// double distance1;
		// double distance2;

		midpt_curr_x = (current_->xRF + current_->xLF)/2;
		midpt_curr_y = (current_->yRF + current_->yLF)/2;

		midpt_goal_x = (goal_->xRF + goal_->xLF)/2;
		midpt_goal_y = (goal_->yRF + goal_->yLF)/2;

		
		//distance = sqrt(pow(neighbor_->xLF - goal_->xLF,2) + pow(neighbor_->yLF - goal_->yLF,2));
		// // }
		// // else{
		// distance2 = sqrt(pow(neighbor_->xRF - goal_->xRF,2) + pow(neighbor_->yRF - goal_->yRF,2));

		distance = sqrt(pow(midpt_curr_x - midpt_goal_x,2) + pow(midpt_curr_y - midpt_goal_y,2));

		if (distance < 0.1){
		//if (abs(current_->xLF - goal_->xLF) < 1.1 && abs(current_->yLF - goal_->yLF) < 1.1 ){
			return true;
		}
		else{
			return false;
		}

	}


	void FootstepPlanner::WriteData(){
		for (size_t i(0); i < optimal_path.size(); i++){
			vector<double> optimal_coord;
			opt_node_ = std::static_pointer_cast<FootstepNode>(optimal_path[i]);
			optimal_coord.push_back(opt_node_->xLF);
			optimal_coord.push_back(opt_node_->yLF);
			optimal_coord.push_back(opt_node_->xRF);
			optimal_coord.push_back(opt_node_->yRF);
			optimal_coord.push_back(opt_node_->thetaLF);
			optimal_coord.push_back(opt_node_->thetaRF);
			saveVector(optimal_coord,"optimal_coordinates_footstep");

		}
	}

	void FootstepPlanner::printPath(){
		for (size_t i(0); i < optimal_path.size(); i++){
			opt_node_ = std::static_pointer_cast<FootstepNode>(optimal_path[i]);
			cout << "optimal path:: " << " xLF: " << opt_node_->xLF << " yLF: " << opt_node_->yLF << " xRF: " << opt_node_->xRF << " yRF: " << opt_node_->yRF << " thetaLF: " << opt_node_->thetaLF << " thetaRF: " << opt_node_->thetaRF << endl;
		}		
	}


	std::vector< shared_ptr<Node> > FootstepPlanner::getNeighbors(shared_ptr<Node> & current){
		current_ = std::static_pointer_cast<FootstepNode>(current);
		std::vector< shared_ptr<Node> > neighbors;

		

		//define discritization
		double x_df = 1.0;
		double y_df = 1.0;
		double theta_df = M_PI/8;
		//define min and max distances
		double mindist_y = 1.0;
		double maxdist_y = 3.0;
		double mindist_x = -2.0;
		double maxdist_x = 2.0;
		double mintheta = -M_PI/4;
		double maxtheta = M_PI/4;

		//bool change = true; //LF moving

		// current_->thetaRF = -M_PI/8;
		//define transform matrix
		Eigen::Matrix4d transform(4,4);
		if (current_->turn == "LF"){
			transform << cos(current_->thetaRF), -sin(current_->thetaRF), 0, current_->xRF,
						sin(current_->thetaRF), cos(current_->thetaRF), 0, current_->yRF,
						0, 0, 1, current_->thetaRF,
						0, 0, 0, 1;
		}
		else if (current_->turn == "RF"){
			transform << cos(current_->thetaLF), -sin(current_->thetaLF), 0, current_->xLF,
						sin(current_->thetaLF), cos(current_->thetaLF), 0, current_->yLF,
						0, 0, 1, current_->thetaLF,
						0, 0, 0, 1;
		}

		//number of points in vector
		int ind_x = int(abs(maxdist_x - mindist_x)/x_df) + 1;
		int ind_y = int(abs(maxdist_y - mindist_y)/y_df) + 1;
		int ind_theta = int(abs(maxtheta - mintheta)/theta_df) + 1;

		//create x and y vectors of points
		vector<double> x_vals;
		vector<double> y_vals;
		vector<double> theta_vals;

		

		if (current_->turn == "LF"){
			for (size_t i(0);i < ind_y ;i++){
				y_vals.push_back(mindist_y + i*y_df);
			}
		}
		else if (current_->turn == "RF"){
			for (size_t i(0);i < ind_y ;i++){
				y_vals.push_back(-mindist_y - i*y_df);
			}
		}
		for (size_t j(0);j < ind_x ;j++){
			x_vals.push_back(mindist_x + j*x_df);
		}


		for (size_t k(0);k < ind_theta;k++){
			theta_vals.push_back(mintheta + k*theta_df);
		}

		

		for (size_t m(0);m < x_vals.size();m++){
			for (size_t n(0);n < y_vals.size();n++){
				for (size_t l(0);l < theta_vals.size();l++){
					Eigen::Vector4d coord_Fframe(x_vals[m],y_vals[n],theta_vals[l],1);
					vector<double> x_y_test;
					x_y_test.push_back(coord_Fframe[0]);
					x_y_test.push_back(coord_Fframe[1]);
					x_y_test.push_back(coord_Fframe[2]);
					saveVector(x_y_test,"x_y_test");
		
					Eigen::Vector4d coord_Oframe = transform*coord_Fframe;
					vector<double> x_y_theta_coord;
					x_y_theta_coord.push_back(coord_Oframe[0]);
					x_y_theta_coord.push_back(coord_Oframe[1]);
					x_y_theta_coord.push_back(coord_Oframe[2]);
					saveVector(x_y_theta_coord,"test_coords");
				
					if (current_->turn == "LF"){
						shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(coord_Oframe[0],coord_Oframe[1],current_->xRF,current_->yRF,coord_Oframe[2],current_->thetaRF,"RF"));
						shared_ptr<FootstepNode> neighbor_change;
						neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
						neighbor_change->parent = current;
						neighbor = static_pointer_cast<Node>(neighbor_change);
						neighbors.push_back(neighbor);
					}
					else if (current_->turn == "RF"){
						shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(current_->xLF,current_->yLF,coord_Oframe[0],coord_Oframe[1],current_->thetaLF,coord_Oframe[2],"LF"));
						shared_ptr<FootstepNode> neighbor_change;
						neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
						neighbor_change->parent = current;
						neighbor = static_pointer_cast<Node>(neighbor_change);
						neighbors.push_back(neighbor);
					}
					//neighbors.push_back(neighbor);	
				}

			}
		}

		// for (size_t s(0);s < neighbors.size();s++){
		// 	shared_ptr<FootstepNode> neighbor_test_;
		// 	neighbor_test_ = static_pointer_cast<FootstepNode>(neighbors[s]);
		// 	vector<double> debug_coords;
		// 	debug_coords.push_back(neighbor_test_->xLF);
		// 	debug_coords.push_back(neighbor_test_->yLF);
		// 	debug_coords.push_back(neighbor_test_->xRF);
		// 	debug_coords.push_back(neighbor_test_->yRF);
		// 	saveVector(debug_coords,"debug_coords"); 
		// }
		//cout << "size of neighbors: " << neighbors.size() << endl;
		return neighbors;
	}




	//************************************************************************************


	// A_starPlanner Implementation ------------------------------
	// Constructor
	A_starPlanner::A_starPlanner(){
		std::cout << "[A_starPlanner Constructed]" << std::endl;
	}
	// Destructor
	A_starPlanner::~A_starPlanner(){
		std::cout << "[A_starPlanner Destroyed]" << std::endl;
	}

	void A_starPlanner::setStartNode(const shared_ptr<Node> begin_input){
		begin = begin_input;
	}

	void A_starPlanner::setGoalNode(const shared_ptr<Node> goal_input){
		goal = goal_input;
	}

	// gScore Implementation: computes the cost from going to the current to the neighbor node
	double A_starPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
		return 0.0;
	}

	// heuristicCost Implementation: computes the cost from going from neighbor node to goal node
	double A_starPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		return 0.0;
	}


	std::vector< shared_ptr<Node> > A_starPlanner::getNeighbors(shared_ptr<Node> & current){
		std::vector< shared_ptr<Node> > neighbors;
		return neighbors;
	}

	//returns the optimal path

	void A_starPlanner::constructPath(){
		// Clear cached optimal path
		optimal_path.clear();
		// Set Current node to the achieved goal
		shared_ptr<Node> current_node = achieved_goal;
		while (begin->key.compare(current_node->key) != 0) {
			optimal_path.push_back(current_node);
			current_node = current_node->parent;	
		}
		// Add the parent
		optimal_path.push_back(current_node);
	}

	bool A_starPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
		return true;
	}


	//perform the A_star algorithm

	bool A_starPlanner::doAstar(){
		NodePtr_Compare_key node_ptr_compare_key_obj;
		NodePtr_Compare_Fcost node_compare_fcost_obj;

		//Initialize starting node with g score and f score
		begin->g_score = 0;
		begin->f_score = heuristicCost(begin,goal);

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
				// cout << "size of open set: " << OpenSet.size() << endl;
				// cout << "size of explored set: " << ExploredSet.size() << endl;
				std::sort(OpenSet.begin(), OpenSet.end(), node_compare_fcost_obj);

				//choose top value of open set as current node;
				std::shared_ptr<FootstepNode> current_;
				current_node = OpenSet[0];
				current_ = std::static_pointer_cast<FootstepNode>(current_node);
				cout << "current node f score: " << current_node->f_score << endl;
				cout << "current node key: " << current_->key << endl;
				cout << "xLF: " << current_->xLF << "yLF: " << current_->yLF << "xRF: " << current_->xRF << "yRF: " << current_->yRF << endl;
				cout << "size of open set: " << OpenSet.size() << endl;
				cout << "turn of current: " << current_-> turn << endl;

				//is current node = goal node?
				if (goalReached(current_node, goal) == true){
					cout << "goal reached" << endl;
					//reproduce path
					// cout << "made it to the end!!!!!" << endl;
					achieved_goal = current_node;
					// Construct the path
					constructPath();
					cout << "completed constructpath" << endl;

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
					ClosedSet.insert(current_node); 

					//create new neighbor nodes
					std::vector< shared_ptr<Node> > neighbors;
					neighbors = getNeighbors(current_node);

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
								double tentative_gscore = current_node->g_score + gScore(current_node,neighbors[i]);
								
								//check to see if g score path is better than previous
								if (tentative_gscore < (*es_it)->g_score){ 

									//update node
									(*es_it)->parent = current_node; 
									(*es_it)->g_score = tentative_gscore; 
									(*es_it)->f_score = tentative_gscore + heuristicCost(neighbors[i],goal); 
								
								}

							}
							else{
								neighbors[i]->g_score = current_node->g_score + gScore(current_node,neighbors[i]);
								neighbors[i]->f_score = neighbors[i]->g_score + heuristicCost(neighbors[i],goal);
								OpenSet.push_back(neighbors[i]);
								ExploredSet.insert(neighbors[i]);
							}		
						}
					}
				}
			

		}
		cout << "did not find a path" << endl;
		return false; // Failed to find a path

	}
}







/*
	std::vector< shared_ptr<Node> > A_starPlanner::getNeighborsFeet(shared_ptr<Node> & current){
		vector<double> x_LF_vals;
		vector<double> y_LF_vals;
		vector<double> x_RF_vals;
		vector<double> y_RF_vals;

		//values of newly created x,y values for both LF and RF
		double x_LF_new;
		double y_LF_new;
		double x_RF_new;
		double y_RF_new;

		int k = -3;
		int f = -2;
		int m = 1;
		int n = -2;

		vector< vector<double> > LF_coords;
		vector< vector<double> > RF_coords;
		vector< shared_ptr<Node> > neighborsLF;
		vector< shared_ptr<Node> > neighborsRF;

		
		while (k < 0){
        	double temp_val_x = 1*k;
        	x_LF_new = current->x + temp_val_x;
        	x_LF_vals.push_back(x_LF_new);
        	k++;
    	}
    	while (f < 3){
        	double temp_val_y = 1*f;
        	y_LF_new = current->y + temp_val_y;
        	y_LF_vals.push_back(y_LF_new);
        	f++;
    	}
    	for (size_t i(0);i < x_LF_vals.size();i++){
        	for (size_t j(0);j < y_LF_vals.size();j++){
           		vector<double> x_y_coord_LF;
           		// x_y_coord_LF.resize(3);
           		x_y_coord_LF.push_back(x_LF_vals[i]);
           		x_y_coord_LF.push_back(y_LF_vals[j]);             
           		saveVector(x_y_coord_LF, "LF_coords");
           		LF_coords.push_back(x_y_coord_LF);
           		shared_ptr<Node> neighborLF(new Node(x_LF_vals[i],y_LF_vals[j]));
           		neighborLF->parent = current;
           		neighborsLF.push_back(neighborLF);
        	}   
    	}



    	while (m < 4){
        	double temp_val_x = 1*m;
        	x_RF_new = current->x + temp_val_x;
        	x_RF_vals.push_back(x_RF_new);
        	m++;
    	}
    	while (n < 3){
        	double temp_val_y = 1*n;
        	y_RF_new = current->y + temp_val_y;
        	y_RF_vals.push_back(y_RF_new);
        	n++;
    	}
    	for (size_t i(0);i < x_RF_vals.size();i++){
        	for (size_t j(0);j < y_RF_vals.size();j++){
           		vector<double> x_y_coord_RF;
           		// x_y_coord_LF.resize(3);
           		x_y_coord_RF.push_back(x_RF_vals[i]);
           		x_y_coord_RF.push_back(y_RF_vals[j]);             
           		saveVector(x_y_coord_RF, "RF_coords");
           		RF_coords.push_back(x_y_coord_RF);
           		shared_ptr<Node> neighborRF(new Node(x_RF_vals[i],y_RF_vals[j]));
           		neighborRF->parent = current;
           		neighborsRF.push_back(neighborRF);
        	}   
    	}
    }
	//Generates neighbors based on current node
*/