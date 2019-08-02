#include <stdio.h>
#include <avatar_locomanipulation/planners/a_star_planner.hpp>

#include <chrono>
#include <ctime>

using namespace planner;

int main(int argc, char **argv){
	A_starPlanner planner;

    FootstepPlanner footstepplanner;

    double lfx_i = -1.0;
    double lfy_i = 1.0;
    double rfx_i = -1.0;
    double rfy_i = -1.0;

    shared_ptr<Node> begin_footstep (std::make_shared<FootstepNode>(lfx_i ,lfy_i, rfx_i, rfy_i, 0.0, 0.0,"RF",0.0));
    double lfx_g = 15.0;
    double lfy_g = 9.0;
    double rfx_g = 15.0;
    double rfy_g = 7.0;
    // shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(lfx_g, lfy_g, rfx_g, rfy_g, -M_PI/4.0, -M_PI/4.0,false));
    shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(lfx_g, lfy_g, rfx_g, rfy_g, M_PI/4, M_PI/4,"RF",0.0));

    footstepplanner.setStartNode(begin_footstep);
    footstepplanner.setGoalNode(goal_footstep);

    footstepplanner.doAstar();

    footstepplanner.printPath();
    footstepplanner.WriteData();


	return 0;
}


