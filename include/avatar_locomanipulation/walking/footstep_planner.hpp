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
#include <Eigen/Dense>
#include <fstream>


using namespace std;

namespace planner{
	class NodePtr_Compare_Fcost;
	class NodePtr_Compare_key;

	class Node {
	public:
		Node(); // Constructor
		virtual ~Node(); // Destructor

		double g_score;
		double f_score;
		shared_ptr<Node> parent;
		string key;

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

	class XYNode: public Node{
	public:
		XYNode(); // Constructor
		virtual ~XYNode(); // Destructor


	// Problem specific:
		XYNode(const double x_in, const double y_in); // Constructor
		double x;
		double y;

		void commonInitialization();

	};


	class XYPlanner: public A_starPlanner{
	public:
		XYPlanner();
		virtual ~XYPlanner();

		virtual	double gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor);
		virtual double heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal);
		virtual bool goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal);

		virtual std::vector< shared_ptr<Node> > getNeighbors(shared_ptr<Node> & current);

		void WriteData(vector< shared_ptr<Node> > optimal_path);
		void printPath();

		int branch;
		double disc;

		std::shared_ptr<XYNode> current_;
		std::shared_ptr<XYNode> goal_;
		std::shared_ptr<XYNode> neighbor_;		
		std::shared_ptr<XYNode> opt_node_;


	};



	class FootstepNode: public Node{
	public:
		FootstepNode(); // Constructor
		virtual ~FootstepNode(); // Destructor


	// Problem specific:
		FootstepNode(const double xLF_in, const double yLF_in, const double xRF_in, const double yRF_in, const double thetaLF_in, const double thetaRF_in, bool turn_in); // Constructor
		double xLF;
		double yLF;
		double xRF;
		double yRF;

		double thetaLF;
		double thetaRF;

		bool turn; //true = LF, false = RF

		double s;

		void commonInitializationFootstep();

	};


	class FootstepPlanner: public A_starPlanner{
	public:
		FootstepPlanner();
		virtual ~FootstepPlanner();

		virtual	double gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor);
		virtual double heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal);
		virtual bool goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal);

		virtual std::vector< shared_ptr<Node> > getNeighbors(shared_ptr<Node> & current);

		void WriteData();
		void printPath();

		int branch;
		double disc;

		std::shared_ptr<FootstepNode> current_;
		std::shared_ptr<FootstepNode> goal_;
		std::shared_ptr<FootstepNode> neighbor_;		
		std::shared_ptr<FootstepNode> opt_node_;

		//neighbor generation
		double x_df;
		double y_df;
		double theta_df;

		double max_reach;
		double max_width;
		double max_angle;
		double min_angle;

		double mindist_reach;
		double maxdist_reach;
		double mindist_width;
		double maxdist_width;
		double mindist_theta;
		double maxdist_theta;


	};


}



#endif