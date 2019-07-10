#ifndef ALM_FOOTSTEP_PLANNER_H
#define ALM_FOOTSTEP_PLANNER_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <string>

using namespace std;

namespace footstep_planner{
	class Node;
	class A_starPlanner;

	class Node {
	public:
		Node(); // Constructor
		Node(const double x_in, const double y_in); // Constructor

		~Node(); // Destructor
		bool obstacle;
		bool visited;
		double GlobalGoal;
		double LocalGoal;
		double f_score;
		double x_R;
		double y_R;
		double x_L;
		double y_L;
		bool turn;
		bool start;
		string parent;
		string key;

		double x;
		double y;

	private:
		void commonInitialization();
	};

	class A_starPlanner{
	public:
		A_starPlanner();
		~A_starPlanner();
		void getPath(std::vector<Node> & path);

		double gScore(const Node & current, const Node & neighbor);
	};


}



#endif