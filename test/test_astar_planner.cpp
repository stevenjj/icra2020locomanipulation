#include <stdio.h>
#include <avatar_locomanipulation/walking/footstep_planner.hpp>

using namespace footstep_planner;

int main(int argc, char **argv){
	A_starPlanner planner;
	Node current(10,10);
	Node neighbor(70,25);

	std::cout << "Gscore between current and neighbor = " << planner.gScore(current, neighbor) << std::endl;

	return 0;
}