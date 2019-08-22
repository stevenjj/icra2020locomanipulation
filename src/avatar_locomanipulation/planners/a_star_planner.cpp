#include <avatar_locomanipulation/planners/a_star_planner.hpp>

// #define THIS_COM "/home/hcappel1/Documents/"
// FILE * pFileTXT;

// 	void saveVector(const std::vector<double>& _vec, std::string _name, bool b_param = false);
// 	void saveValue(double _value, std::string _name, bool b_param = false);
// 	void cleaningFile(std::string file_name_, std::string& ret_file_, bool b_param);
// 	static std::list<std::string> gs_fileName_string;  // global & static

// 	void saveVector(const std::vector<double>& _vec, std::string _name, bool b_param) {
//     	std::string file_name;
//     	cleaningFile(_name, file_name, b_param);
//     	std::ofstream savefile(file_name.c_str(), std::ios::app);
//     	for (int i(0); i < _vec.size(); ++i) {
//         	savefile << _vec[i] << "\t";
//     	}
//     	savefile << "\n";
//     	savefile.flush();
// 	}

// 	void cleaningFile(std::string _file_name, std::string& _ret_file, bool b_param) {
//     	if (b_param)
//         	_ret_file += THIS_COM;
//     	else
//         	_ret_file += THIS_COM;

//     	_ret_file += _file_name;
//     	_ret_file += ".txt";

//     	std::list<std::string>::iterator iter = std::find(
//         	gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
//     	if (gs_fileName_string.end() == iter) {
//         	gs_fileName_string.push_back(_file_name);
//         	remove(_ret_file.c_str());
//     	}
// 	}





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
			
	}

	// Destructor
	Node::~Node(){
			
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
			//saveVector(optimal_coord,"optimal_path");
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
				//saveVector(x_y_coord,"foot coordinates");
				coords.push_back(x_y_coord);
				shared_ptr<Node> neighbor(std::make_shared<XYNode>(x_vals[i],y_vals[j]));
				neighbor->parent = current_;
				neighbors.push_back(neighbor);
			}
		}
		return neighbors;
	}



	FootstepNode::FootstepNode(){		
	commonInitializationFootstep();
	}
	FootstepNode::~FootstepNode(){		
	}

	FootstepNode::FootstepNode(const double xLF_in,const double yLF_in,const double xRF_in,const double yRF_in,const double thetaLF_in, const double thetaRF_in, string turn_in, double s_in){
		commonInitializationFootstep();
		xLF = xLF_in;
		yLF = yLF_in;
		xRF = xRF_in;
		yRF = yRF_in;

		thetaLF = thetaLF_in;
		thetaRF = thetaRF_in;

		turn = turn_in;

		s = s_in;
		

		key = (to_string((int)round(xLF*1000)) + "_" + to_string((int)round(yLF*1000)) + "_" + to_string((int)round(xRF*1000)) + "_" + to_string((int)round(yRF*1000)) + "_" + to_string(thetaLF) + "_" + to_string(thetaRF) + "_" + (turn) + "_" + to_string(s));	
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

		s = 0.0;
		step_num = 0;

		key = (to_string((int)round(xLF*1000)) + "_" + to_string((int)round(yLF*1000)) + "_" + to_string((int)round(xRF*1000)) + "_" + to_string((int)round(yRF*1000)) + "_" + to_string(thetaLF) + "_" + to_string(thetaRF) + "_" + (turn) + "_" + to_string(s));	

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

		double distance;

		double left_foot_transition_cost = sqrt(pow(current_->xLF - neighbor_->xLF,2) + pow(current_->yLF - neighbor_->yLF,2) + pow(current_->thetaLF - neighbor_->thetaLF,2)); 
		double right_foot_transition_cost = sqrt(pow(current_->xRF - neighbor_->xRF,2) + pow(current_->yRF - neighbor_->yRF,2) + pow(current_->thetaRF - neighbor_->thetaRF,2)); 

		if ((left_foot_transition_cost > 1e-12) || (right_foot_transition_cost > 1e-12)){
			distance = 10 + (1 -  neighbor_->s);
		}else{
			distance = (1 - neighbor_->s);
		}

		
		return distance;
	}

	double FootstepPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		goal_ = std::static_pointer_cast<FootstepNode>(goal);
		neighbor_ = std::static_pointer_cast<FootstepNode>(neighbor);

		double distance;

		double left_foot_cost = sqrt(pow(goal_->xLF - neighbor_->xLF,2) + pow(goal_->yLF - neighbor_->yLF,2) + pow(goal_->thetaLF - neighbor_->thetaLF,2));
		double right_foot_cost = sqrt(pow(goal_->xRF - neighbor_->xRF,2) + pow(goal_->yRF - neighbor_->yRF,2) + pow(goal_->thetaRF - neighbor_->thetaRF,2));

		distance =  20*(1.0 -  neighbor_->s); 



		return distance;
	}

	bool FootstepPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
		current_ = std::static_pointer_cast<FootstepNode>(current_node);
		goal_ = std::static_pointer_cast<FootstepNode>(goal);

		double left_foot_cost = sqrt(pow(goal_->xLF - current_->xLF,2) + pow(goal_->yLF - current_->yLF,2) + pow(goal_->thetaLF - current_->thetaLF,2));
		double right_foot_cost = sqrt(pow(goal_->xRF - current_->xRF,2) + pow(goal_->yRF - current_->yRF,2) + pow(goal_->thetaRF - current_->thetaRF,2));
	
		if (fabs(1.0 - current_->s) < 0.1){
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
			//saveVector(optimal_coord,"optimal_coordinates_footstep");

		}
	}

	void FootstepPlanner::printPath(){
		for (size_t i(0); i < optimal_path.size(); i++){
			opt_node_ = std::static_pointer_cast<FootstepNode>(optimal_path[i]);
			cout << "optimal path:: " << " xLF: " << opt_node_->xLF << " yLF: " << opt_node_->yLF << " xRF: " << opt_node_->xRF << " yRF: " << opt_node_->yRF << " thetaLF: " << opt_node_->thetaLF << " thetaRF: " << opt_node_->thetaRF << " s value: " << opt_node_->s << " footstep number: " << opt_node_->step_num << endl;
		}		
	}


	std::vector< shared_ptr<Node> > FootstepPlanner::getNeighbors(shared_ptr<Node> & current){
		current_ = std::static_pointer_cast<FootstepNode>(current);
		std::vector< shared_ptr<Node> > neighbors;


		x_df = 0.05;
		y_df = 0.05;
		theta_df = 10.0*M_PI/180.0;

		maxdist_reach = 0.4;
		mindist_reach =  -0.2;

		maxdist_width = 0.4; 
		mindist_width = 0.2;

		maxdist_theta = 0.6;
		mindist_theta = -0.15;


		max_reach = maxdist_reach; 
		max_width = maxdist_width; 
		max_angle = maxdist_theta; 
		min_angle = mindist_theta; 


		vector<double> s_vals;
		s_vals.push_back(0.0);
		s_vals.push_back(0.01);
		s_vals.push_back(0.05);
		s_vals.push_back(0.1);


		//number of bins
		int ind_x = int(abs(2*max_reach)/x_df) + 1;
		int ind_y = int(abs(2*max_width)/y_df) + 1;
		int ind_theta = int(abs(max_angle - min_angle)/theta_df);

		vector<double> x_vals;
		vector<double> y_vals;
		vector<double> theta_vals;

		Eigen::Matrix3d transform(3,3);

		//generate theta vals
		for (size_t k(0); k < ind_theta; k++){
			theta_vals.push_back(min_angle + k*theta_df);
		}

		if (current_->turn == "LF"){

			transform << cos(current_->thetaRF), -sin(current_->thetaRF), current_->xRF,
					sin(current_->thetaRF), cos(current_->thetaRF), current_->yRF,
					0, 0, 1;


			for (size_t i(0);i < ind_x; i++){
				x_vals.push_back(current_->xRF - max_reach + i*x_df);
		
			}
			for (size_t j(0);j < ind_y; j++){
				y_vals.push_back(current_->yRF - max_width + j*y_df);
			}
		}
		else if (current_->turn == "RF"){

			transform << cos(current_->thetaLF), -sin(current_->thetaLF), current_->xLF,
					sin(current_->thetaLF), cos(current_->thetaLF), current_->yLF,
					0, 0, 1;



			for (size_t i(0);i < ind_x; i++){
				x_vals.push_back(current_->xLF - max_reach + i*x_df);
				
			}
			for (size_t j(0);j < ind_y; j++){
				y_vals.push_back(current_->yLF - max_width + j*y_df);
				
			}
				
		}

		shared_ptr<FootstepNode> neighbor_change;

		if (current_->step_num < 10){
			for (size_t n(0); n < x_vals.size(); n++){
				for (size_t m(0); m < y_vals.size(); m++){
					for (size_t k(0); k < theta_vals.size(); k++){
						for (size_t l(0); l < s_vals.size(); l++){
							if (s_vals[l] + current_->s < 1){
								Eigen::Vector3d coord_Oframe(x_vals[n],y_vals[m],1);
								Eigen::Vector3d coord_Fframe = transform.inverse()*coord_Oframe;

								if (current_->turn == "LF"){
									if (abs(theta_vals[k] - current_->thetaRF) < maxdist_theta){
										if (coord_Fframe[0] > mindist_reach && coord_Fframe[0] < maxdist_reach && coord_Fframe[1] > mindist_width && coord_Fframe[1] < maxdist_width){
											shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(coord_Oframe[0],coord_Oframe[1],current_->xRF,current_->yRF,theta_vals[k],current_->thetaRF,"RF",s_vals[l] + current_->s));
											neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
											neighbor_change->parent = current;
											neighbor_change->step_num = current_->step_num + 1;
											neighbor = static_pointer_cast<Node>(neighbor_change);
											neighbors.push_back(neighbor);
										}
									}
								}

								else if (current_->turn == "RF"){
									if (abs(theta_vals[k] - current_->thetaLF) < maxdist_theta){
										if (coord_Fframe[0] > mindist_reach && coord_Fframe[0] < maxdist_reach && coord_Fframe[1] < -mindist_width && coord_Fframe[1] > -maxdist_width){
											shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(current_->xLF,current_->yLF,coord_Oframe[0],coord_Oframe[1],current_->thetaLF,theta_vals[k],"LF",s_vals[l] + current_->s));
											neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
											neighbor_change->parent = current;
											neighbor_change->step_num = current_->step_num + 1;
											neighbor = static_pointer_cast<Node>(neighbor_change);
											neighbors.push_back(neighbor);
										}
									}
								}
							}
						}
					}
				}
			}
		}
		
		// The neighbors when we don't take a step
		for (size_t l(1); l < s_vals.size(); l++){
			shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(current_->xLF,current_->yLF, current_->xRF,current_->yRF,current_->thetaLF,current_->thetaRF, current_->turn, s_vals[l] + current_->s));
			neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
			neighbor_change->parent = current;
			neighbor_change->step_num = current_->step_num;
			neighbor = static_pointer_cast<Node>(neighbor_change);
			neighbors.push_back(neighbor);
		}

		return neighbors;
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

	bool A_starPlanner::constructPath(){
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

		return true;
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

		std::set<shared_ptr<Node>, NodePtr_Compare_key> ClosedSet;

		std::set< std::shared_ptr<Node>, NodePtr_Compare_key> ExploredSet;
	
		std::vector< shared_ptr<Node> >::iterator node_it;
		std::set< std::shared_ptr<Node> >::iterator es_it;
		std::set< shared_ptr<Node> >::iterator node_set_it;
		
		OpenSet.push_back(begin); //append starting node to open set

		while (!OpenSet.empty()){
				//sort the open set
				std::sort(OpenSet.begin(), OpenSet.end(), node_compare_fcost_obj);

				//choose top value of open set as current node;
				current_node = OpenSet[0];

				// shared_ptr<FootstepNode> current_ = static_pointer_cast<FootstepNode>(current_node);
				// cout << "current node::" << " xLF: " << current_->xLF << " yLF: " << current_->yLF << " xRF: " << current_->xRF << " yRF: " << current_->yRF << " thetaLF: " << current_->thetaLF << " thetaRF: " << current_->thetaRF << " fscore: " << current_->f_score << " left or right: " << current_->turn << " s value: " << current_->s << endl;
				// cout << "current node key: " << current_->key << endl;
				// cout << "step number: " << current_->step_num << endl;
				// cout << endl;

				//current node = goal node
				if (goalReached(current_node, goal) == true){
					cout << "goal reached" << endl;
					//reproduce path
					achieved_goal = current_node;
					// Construct the path
					constructPath();

					return true;
				}

				//current node not equal to goal

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

					//check if node is not in the closed set
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
								(*es_it)->step_num = neighbors[i]->step_num; 
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
		cout << "did not find a path" << endl;
		return false; // Failed to find a path

	}
}







