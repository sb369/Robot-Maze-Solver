#include "math.h"
#include <array>
#include <string.h>
#include <queue>
#include <iostream>
#include <stack>
#include <stdio.h>
#include <limits>
#include <cfloat>
#include <chrono>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <planner/my_msg.h> // to publish
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


using namespace std;
using std::string;
#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

struct Node{ // info about a particular cell used for finding the path
    int parent;
    double f,g; // f = g+h. removed h from here(not req to store)

    Node(){ //constructor to assign with default values
        parent = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        //h = DBL_MAX;
    }
};

struct Index{
    int pos,orientation;

    Index(){

    }

    Index(int pos, int orientation){
        this->pos = pos;
        this->orientation = orientation;
    }
};

struct ActualPos{
    double x,y,orientation; // exact x,y and angle in rads
};

struct Index_smoothPath{
    int pos,orientation;
    double dd,steer_angle;
    ActualPos pose;
};

struct Node_park{
    Index parent;
    double f,g;

    Node_park(){
        parent.pos = -1;
        parent.orientation = -1;
        f = DBL_MAX;
        g = DBL_MAX;
    }
};

struct Node_park_smoothPath{
    Index parent;
    double f,g;
    double dd,steer_angle;
    ActualPos pose;

    Node_park_smoothPath(){
        parent.pos = -1;
        parent.orientation = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        dd = 0.0;
        steer_angle = 0.0;
        pose.x = -1;
        pose.y = -1;
        pose.orientation = -1;
    }
};

struct Node_local{
    int parent, orientation;
    double f,g;

    Node_local(){
        parent = -1;
        orientation = -1;
        f = DBL_MAX;
        g = DBL_MAX;
    }
};

struct Node_big{ // info about a particular cell used for finding the path
    Index parent;
    double f,g,vel_r,vel_l; // f = g+h. removed h from here(not req to store)

    Node_big(){ //constructor to assign with default values
        parent.pos = -1;
        parent.orientation = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        //h = DBL_MAX;
    }
};

struct Neighbour_smooth{
    int pos,orientation;
    double d,c,V_l,V_r;
};

typedef tuple<double, int, int> Tuple;

typedef pair<double, int> Pair;

namespace globalplanner{
   /* class GlobalPlanner : public nav_core::BaseGlobalPlanner{
        public:
            float originX;
            float originY;
            float resolution;
            int width;
            int height;
            bool initialized_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            ros::NodeHandle my_nh;
            ros::Publisher pubb;


            GlobalPlanner();
            GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            GlobalPlanner(ros::NodeHandle& n);
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
            
            
            
            

            
            

            void getCoordinate(float& x, float& y);
            bool isPointInside(float x, float y);
            int convertToCellIndex(float x, float y);

            int getCellIndex(int i, int j){ // pass (i,j)=(y,x)
                return (width*i + j);
            }
    };*/

    class PlannerAlgorithm{
        public:
            int width, height, maplen;
            vector<int> costmap_;

            PlannerAlgorithm(int w,int h,int len,const vector<int>& costmap){
                width = w;
                height = h;
                maplen = len;
                costmap_ = costmap;
            }
            bool isValid(const int& p);
            bool isNotBlocked(const int& p);
            double distance(const int& src, const int& dest);
            double distance_squared(const int& src, const int& dest);
	        double diagonalDist(const int& src, const int& dest);
            double manhattanDist(const int& src, const int& dest);
            double heuristic(const int& src, const int& dest);
            vector<int> findNeighbours(int index);
            vector<int> findpath(const vector<Node>& matrix, const int& dest);
            double kernel_cost(int index, int ksize);
            double avg_kernel_cost(int index, int ksize);
            bool isInside(const int& row, const int& col);
    };
    class HAstar_park : public PlannerAlgorithm{ //Hybrid Astar for parking
        public:
            double diss = sqrt(2.0) + 0.01;
            // double dd[1] = {diss}; //small dist which should be greater than diagonal of unit cell of map so that neighbours do not fall in the same cell again
            // double steer_angles[3] = {-M_PI/4,0,M_PI/4}; //angles at which car can steer
            double dd[2] = {diss,-diss}; //small dist which should be greater than diagonal of unit cell of map so that neighbours do not fall in the same cell again
            double steer_angles[7] = {-M_PI/3,-M_PI/4,-M_PI/6,0,M_PI/6,M_PI/4,M_PI/3}; //angles at which car can steer
            int L = 10; // distance between front and rear axle of the car

            int orientationRes = 5; //orientation resolution

            //closeness parameters
            int dest_close_x = 1;
            int dest_close_y = 1;
            int dest_close_theta = 10;

            HAstar_park(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}
            vector<Index_smoothPath> search_path(const Index& src, const Index& dest);
            vector<Index_smoothPath> findNeighbours(const ActualPos& index);
            int get_int_pos(const double& x);
            bool dest_reached(const Index& neighbour, const Index& dest);
            vector<Index_smoothPath> findpath(const vector<vector<Node_park_smoothPath>>& matrix, const Index& dest);
    };
    class HAstar_local : public HAstar_park{ //Hybrid Astar for local planning
        public:
            HAstar_local(int w,int h,int len,const vector<int>& costmap)
                : HAstar_park(w,h,len,costmap)
            {}
            vector<Index> search_path(const Index& src, const Index& dest);
            vector<Index> findpath(const vector<Node_local>& matrix, const Index& dest);
    };
    class Astar : public PlannerAlgorithm{
        public:
            Astar(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            
    };
    class RAstar : public PlannerAlgorithm{
        public:
            RAstar(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            vector<int> tracePath(const int& src,const int& dest,const vector<double>& g_score);

    };
    class Tstar : public PlannerAlgorithm{
        public:
            Tstar(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            double line_of_sight(const int& s, const int& s1);
            
    };
};
#endif
