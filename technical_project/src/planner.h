#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Point.h"
#include "iostream"
#include "math.h"
#include "vector"
#include "std_msgs/Float64MultiArray.h"
#include "tf/tf.h"
#include "std_msgs/Float64.h"
#include "fstream"


#define INF 1e10
using namespace std;



class PLANNER{
	public:
		PLANNER();


		struct rrtNode{
			int nodeID;
			double posX;
			double posY;
			int parentID;
			double cost;
			
		};

		void odometry_cb(nav_msgs::Odometry odom);
		void map_cb(nav_msgs::OccupancyGrid map);
		void makePlan();
		void run1();
		void run2();
		void run3();
		void run4();
		void initNode(PLANNER::rrtNode &Node, const geometry_msgs::PoseStamped& posNode);
		rrtNode generateRandNode();
		rrtNode checkNearNode(PLANNER::rrtNode& Node1, vector<PLANNER::rrtNode>& Tree);
		rrtNode checkNewNode(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2);
		bool goalNodeReached(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2);
		double getDistance(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2);
		double getDistance(geometry_msgs::Point& Point, PLANNER::rrtNode& Node);
		double getDistance(geometry_msgs::Point& Point1, geometry_msgs::Point& Point2);
		rrtNode setDistance(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2, double delta);
		void swipe(geometry_msgs::PoseStamped& Node1, PLANNER::rrtNode& Node2);
		bool obstacleFound(PLANNER::rrtNode &Node);
		vector<PLANNER::rrtNode> checkFinalPath(vector<PLANNER::rrtNode>& Tree);
		vector<PLANNER::rrtNode> checkFinalPathStar(vector<PLANNER::rrtNode>& Tree);
		void makePlanBid();
		void makePlanBidStar();
		vector<PLANNER::rrtNode> reverse(vector<PLANNER::rrtNode>& Tree1);
		vector<PLANNER::rrtNode> concatenate(vector<PLANNER::rrtNode>& Tree1, vector<PLANNER::rrtNode>& Tree2);
		void makePlanStar();
		bool isaNeighbour(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2, double radius);
		void readMapData();
		bool pointNotPresent(geometry_msgs::Point& point, vector<geometry_msgs::Point>& pointVec);
		void initParAr();

		
		
	private:

		ros::NodeHandle _nh;
		ros::Subscriber _map_sub;
		vector<geometry_msgs::PoseStamped> _partenza;
		vector<geometry_msgs::PoseStamped> _arrivo;
		std_msgs::Float64MultiArray _obst;
		std_msgs::Float64MultiArray _paths;
		vector<std_msgs::Float64MultiArray> _pathVec;
		nav_msgs::OccupancyGrid _map;
		bool _read_map;
		vector<geometry_msgs::Point> _blackPoints;
		int _nPaths;

		rrtNode randNode;
   		rrtNode nearNode;
    	rrtNode newNode;
    	rrtNode startNode;
    	rrtNode goalNode;
		rrtNode randNodeDir;
		rrtNode randNodeRev;
   		rrtNode nearNodeDir;
		rrtNode nearNodeRev;
		rrtNode newNodeDir;
		rrtNode newNodeRev;
		vector<PLANNER::rrtNode> rrtTree;
		vector<PLANNER::rrtNode> finalPath;
		vector<PLANNER::rrtNode> rrtTreeDir;
		vector<PLANNER::rrtNode> rrtTreeRev;
		vector<PLANNER::rrtNode> rrtTreeBid;
		vector<PLANNER::rrtNode> finalPathDir;
		vector<PLANNER::rrtNode> finalPathRev;
		vector<PLANNER::rrtNode> finalPathBid;
		vector<PLANNER::rrtNode> finalPathStar;
		vector<PLANNER::rrtNode> finalPathBidStar;
		std_msgs::Float64MultiArray path;
		std_msgs::Float64MultiArray pathDir;
		std_msgs::Float64MultiArray pathRev;


};
