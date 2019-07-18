#ifndef ALM_FOOTSTEP_PLANNER_H
#define ALM_FOOTSTEP_PLANNER_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <algorithm>
#include <set>
#include <stdlib.h>
#include <memory>
#include <cstdlib>
#include <sstream>

using namespace std;

namespace planner{
	class NodePtr_Compare_Fcost;
	class NodePtr_Compare_key;

	class Node {
	public:
		Node(); // Constructor
		~Node(); // Destructor

		double g_score;
		double f_score;
		shared_ptr<Node> parent;
		string key;

	// Problem specific:
		Node(const double x_in, const double y_in); // Constructor
		double x;
		double y;

	private:
		void commonInitialization();

	};

	
	class NodePtr_Compare_Fcost{
	public:
		NodePtr_Compare_Fcost();
		bool operator() (const shared_ptr<Node> & lhs, const shared_ptr<Node> &rhs) const;
	};

	class NodePtr_Compare_key{
	public:
		NodePtr_Compare_key();
		bool operator() (const shared_ptr<Node> & lhs, const shared_ptr<Node> &rhs) const;
	};

	class A_starPlanner{
	public:
		A_starPlanner();
		virtual ~A_starPlanner();

		void constructPath();
		bool doAstar();

		void setStartNode(const shared_ptr<Node> begin_input);
		void setGoalNode(const shared_ptr<Node> goal_input);

		virtual	double gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor);
		virtual double heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal);
		virtual bool goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal);
		virtual std::vector< shared_ptr<Node> > getNeighbors(shared_ptr<Node> & current);

		shared_ptr<Node> begin; 
		shared_ptr<Node> goal;		
		shared_ptr<Node> achieved_goal;		
		std::vector< shared_ptr<Node> > optimal_path;	

	};


	class FootstepPlanner: public A_starPlanner{
	public:
		FootstepPlanner();
		virtual ~FootstepPlanner();

		virtual	double gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor);
		virtual double heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal);
		virtual bool goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal);

		virtual std::vector< shared_ptr<Node> > getNeighbors(shared_ptr<Node> & current);

		void WriteData(vector< shared_ptr<Node> > optimal_path);
		void printPath();

		int branch;
		double disc;

	};


}



#endif