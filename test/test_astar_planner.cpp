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
    // // std::shared_ptr<Node> current_test_node;
    // // shared_ptr<FootstepNode> neighbor_test;
    // shared_ptr<FootstepNode> goal_test;


    //shared_ptr<Node> current (std::make_shared<FootstepNode>(5.0,4.0,5.0,3.0,M_PI/4,M_PI/4,false));
    // shared_ptr<Node> goal (std::make_shared<FootstepNode>(-5.0,-20.0,-5.0,-22.0,0.0,0.0,false));
    // current_test = std::static_pointer_cast<FootstepNode>(current);
    // goal_test = std::static_pointer_cast<FootstepNode>(goal);



    // footstepplanner.heuristicCost(current_test,goal_test);

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

    // // shared_ptr<Node> begin_footstep (std::make_shared<FootstepNode>(1.0,1.0,1.0,-1.0,M_PI/8,M_PI/8,true));
    // // shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(7.0,-7.0,7.0,-9.0,-M_PI/4,-M_PI/8,false));

    double lfx_i = -1.0;
    double lfy_i = 1.0;
    double rfx_i = -1.0;
    double rfy_i = -1.0;

    shared_ptr<Node> begin_footstep (std::make_shared<FootstepNode>(lfx_i ,lfy_i, rfx_i, rfy_i, 0.0, 0.0,false,0.0));
    double lfx_g = 15.0;
    double lfy_g = 9.0;
    double rfx_g = 15.0;
    double rfy_g = 7.0;
    // shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(lfx_g, lfy_g, rfx_g, rfy_g, -M_PI/4.0, -M_PI/4.0,false));
    shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(lfx_g, lfy_g, rfx_g, rfy_g, M_PI/4, M_PI/4,false,0.0));

    footstepplanner.setStartNode(begin_footstep);
    footstepplanner.setGoalNode(goal_footstep);

    footstepplanner.doAstar();

    footstepplanner.printPath();
    footstepplanner.WriteData();


	return 0;
}






		// double mindist_y = 1.0;
		// double maxdist_y = 3.0;
		// double mindist_x = -2.0;
		// double maxdist_x = 2.0;
		// double mintheta = -M_PI/4.0;
		// double maxtheta = M_PI/4.0;

		// //define transform matrix
		// Eigen::Matrix3d transform(3,3);
		// if (current_->turn == true){
			// transform << cos(current_->thetaRF), -sin(current_->thetaRF), current_->xRF,
			// 		sin(current_->thetaRF), cos(current_->thetaRF), current_->yRF,
			// 		0, 0, 1;
		// }
		// else if (current_->turn == false){
		// 	transform << cos(current_->thetaLF), -sin(current_->thetaLF), current_->xLF,
		// 			sin(current_->thetaLF), cos(current_->thetaLF), current_->yLF,
		// 	 		0, 0, 1;
		// }

		// //number of points in vector
		// int ind_x = int(abs(maxdist_x - mindist_x)/x_df) + 1;
		// int ind_y = int(abs(maxdist_y - mindist_y)/y_df) + 1;
		// int ind_theta = int(abs(maxtheta - mintheta)/theta_df) + 1;

		// //create x and y vectors of points
		// vector<double> x_vals;
		// vector<double> y_vals;
		// vector<double> theta_vals;

		

		// if (current_->turn == true){
		// 	for (size_t i(0);i < ind_y ;i++){
		// 		y_vals.push_back(mindist_y + i*y_df);
		// 	}
		// }
		// else if (current_->turn == false){
		// 	for (size_t i(0);i < ind_y ;i++){
		// 		y_vals.push_back(-mindist_y - i*y_df);
		// 	}
		// }
		// for (size_t j(0);j < ind_x ;j++){
		// 	x_vals.push_back(mindist_x + j*x_df);
		// }


		// for (size_t k(0);k < ind_theta;k++){
		// 	theta_vals.push_back(mintheta + k*theta_df);
		// }

		

		// for (size_t m(0);m < x_vals.size();m++){
		// 	for (size_t n(0);n < y_vals.size();n++){
		// 		for (size_t l(0);l < theta_vals.size();l++){
		// 			Eigen::Vector3d coord_Fframe(x_vals[m],y_vals[n],1);
		// 			vector<double> x_y_test;
		// 			x_y_test.push_back(coord_Fframe[0]);
		// 			x_y_test.push_back(coord_Fframe[1]);
		// 			x_y_test.push_back(theta_vals[l]);
		// 			saveVector(x_y_test,"x_y_test");
		
		// 			Eigen::Vector3d coord_Oframe = transform*coord_Fframe;
		// 			vector<double> x_y_theta_coord;
		// 			x_y_theta_coord.push_back(coord_Oframe[0]);
		// 			x_y_theta_coord.push_back(coord_Oframe[1]);
		// 			if (current_->turn == true){
		// 				x_y_theta_coord.push_back(current_->thetaRF + theta_vals[l]);
		// 			}
		// 			else{
		// 				x_y_theta_coord.push_back(current_->thetaLF + theta_vals[l]);
		// 			}
		// 			saveVector(x_y_theta_coord,"test_coords");
				
		// 			if (current_->turn == true){
						
		// 				double theta_node;
		// 				if (current_->thetaRF + theta_vals[l] > M_PI){
		// 					theta_node = current_->thetaRF + theta_vals[l] - 2*M_PI;
		// 				}
		// 				else if (current_->thetaRF + theta_vals[l] < -M_PI){
		// 					theta_node = current_->thetaRF + theta_vals[l] + 2*M_PI;
		// 				}
		// 				else{
		// 					theta_node = current_->thetaRF + theta_vals[l];
		// 				}
		// 				shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(round(coord_Oframe[0]*1000.0)/1000.0,round(coord_Oframe[1]*1000.0)/1000.0,current_->xRF,current_->yRF,theta_vals[l],current_->thetaRF,false));
		// 				shared_ptr<FootstepNode> neighbor_change;
		// 				neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
		// 				neighbor_change->parent = current;
		// 				neighbor = static_pointer_cast<Node>(neighbor_change);
		// 				neighbors.push_back(neighbor);
		// 			}
		// 			else if (current_->turn == false){
						
		// 				double theta_node;
		// 				if (current_->thetaLF + theta_vals[l] > M_PI){
		// 					theta_node = current_->thetaLF + theta_vals[l] - 2*M_PI;
		// 				}
		// 				else if (current_->thetaLF + theta_vals[l] < -M_PI){
		// 					theta_node = current_->thetaLF + theta_vals[l] + 2*M_PI;
		// 				}
		// 				else{
		// 					theta_node = current_->thetaLF + theta_vals[l];
		// 				}
		// 				shared_ptr<Node> neighbor (std::make_shared<FootstepNode>(current_->xLF,current_->yLF,round(coord_Oframe[0]*1000.0)/1000.0,round(coord_Oframe[1]*1000.0)/1000.0,current_->thetaLF,theta_vals[l],true));
		// 				shared_ptr<FootstepNode> neighbor_change;
		// 				neighbor_change = static_pointer_cast<FootstepNode>(neighbor);
		// 				neighbor_change->parent = current;
		// 				neighbor = static_pointer_cast<Node>(neighbor_change);
		// 				neighbors.push_back(neighbor);
		// 			}	
		// 		}
		// 	}
		// }