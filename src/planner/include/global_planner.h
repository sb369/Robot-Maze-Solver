#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <math.h>
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
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>


using namespace std;
using std::string;
// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

/**
 * A struct to represent a Node
 * @param parent <int> Index of parent node in a 2d map
 * @param f <double> f_cost of the node
 * @param g <double> g_cost of the node
 * @see Node
 */
struct Node{ 
    int parent;
    double f,g; // f = g+h. removed h from here(not req to store)

    Node(){ //constructor to assign with default values
        parent = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        //h = DBL_MAX;
    }
};

/**
 * A struct to represent a cell
 * @param pos <int> index/coordinates of the cell in a 2d map
 * @param orientation <int> angle/orientation at this cell
 */
struct Index{
    int pos,orientation;

    Index(){

    }

    Index(int pos, int orientation){
        this->pos = pos;
        this->orientation = orientation;
    }
};

/**
 * A struct to store exact x,y and angle in rads
 */
struct ActualPos{
    double x,y,orientation; 
};

/**
 * A struct to represent a cell with more info(angle, actual pose etc)
 */
struct Index_smoothPath{
    int pos,orientation;
    double dd,steer_angle;
    ActualPos pose;
};

/**
 * A struct to represent a Node in car parking
 * @param parent  <Index> index/coordinates & orientation of the parent cell
 * @param f <double> f_cost
 * @param g <double> g_cost
 */
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

/**
 * A struct to represent a Node in car parking
 * @param parent  <Index> index/coordinates & orientation of the parent cell
 * @param f <double> f_cost
 * @param g <double> g_cost
 * @param dd <double> drive distance
 * @param steer_angle <double> steer_angle to reach the cell
 * @param pose <ActualPose> non-approximated x,y,theta
 */
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

/**
 * Node for local planning of car
 * @param parent <int> coordinate/index of parent cell
 * @param orientation <int> orientation/pose of parent cell
 * @param f <double> f_cost
 * @param g <double> g_cost
 */
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

struct Node_big{ //Ignore For Now
    // info about a particular cell used for finding the path
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

struct Neighbour_smooth{ //Ignore For Now
    int pos,orientation;
    double d,c,V_l,V_r;
};

typedef pair<double, int> Pair;
typedef tuple<double, int, int> Tuple;


namespace globalplanner{

    /**
     * @class GlobalPlanner
     * GlobalPlanner plugin class
     * @param originX <float> x-origin of map
     * @param originY <float> y-origin of map
     * @param width <int> width of map
     * @param height <int> height of map
     * @param resolution <float> map resolution
     * @param costmap_ pointer to the ros costmap
     */
    class GlobalPlanner : public nav_core::BaseGlobalPlanner{
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

            int iter=0,prev_goal,checkpt_dist=300,checkpoint;
            float temp_dist=0,full_dist=0;

            float accn,d_robot;
            Index smooth_prev_start;
            int time_frame;

            /**
             * GlobalPlanner class constructor
             */
            GlobalPlanner();

            /**
             * GlobalPlanner class constructor
             */
            GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            GlobalPlanner(ros::NodeHandle& n);

            /**
             * Initialize class params
             * @param name initializer
             * @param costmap_ros ros costmap
             */
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            /**
             * Function to find path
             * @param start start pose
             * @param goal goal pose
             * @param plan path vector is calculated and stored in this variable
             */
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
            
            
            
            

            
            
            /**
             * Subtracting origin
             * @param x <float>
             * @param y <float>
             */
            void getCoordinate(float& x, float& y);

            /**
             * Check if point is inside map
             * @param x <float>
             * @param y <float>
             */
            bool isPointInside(float x, float y);

            /**
             * convert world to map
             * @param x <float>
             * @param y <float>
             */
            int convertToCellIndex(float x, float y);

            /**
             * convert i&j of 2d matrix to single index
             * @param i <int> row
             * @param j <int >col
             */
            int getCellIndex(int i, int j){ // pass (i,j)=(y,x)
                return (width*i + j);
            }
    };


    // /**
    //  * PlannerAlgorithm: A general template class for all planners
    //  * @param width <int> width of the map
    //  * @param height <int> height of the map
    //  * @param maplen <int> size of map
    //  * @param costmap_ <costmap_2d::Costmap2D> costmap 
    //  */
    // class PlannerAlgorithm{
    //     public:
    //         int width, height, maplen;
    //         costmap_2d::Costmap2D* costmap_;

    //         PlannerAlgorithm(int w,int h,int len,costmap_2d::Costmap2D* costmap){
    //             width = w;
    //             height = h;
    //             maplen = len;
    //             costmap_ = costmap;
    //         }

    //         /**
    //          * If cell lies inside map
    //          * @param p <int> index of 2d map
    //          */
    //         bool isValid(const int& p);

    //         /**
    //          * Check if the cell is not an obstacle or not
    //          * @param p <int> index of 2d map
    //          */
    //         bool isNotBlocked(const int& p);

    //         /**
    //          * Euclidiean distance between two points
    //          * @param src <int> index of 2d map
    //          * @param dest <int> index of 2d map
    //          * @returns <double> euclidean distance between two points
    //          */
    //         double distance(const int& src, const int& dest);

    //         /**
    //          * square of distance between two points
    //          * @param src <int> index of 2d map
    //          * @param dest <int> index of 2d map
    //          * @returns <double> square of distance between two points
    //          */
    //         double distance_squared(const int& src, const int& dest);

    //         /**
    //          * distance between two points 
    //          * 1 for adjacent, 1.4142 for diagonal
    //          * @param src <int> index of 2d map
    //          * @param dest <int> index of 2d map
    //          * @returns <double> distance between two points
    //          */
	//         double diagonalDist(const int& src, const int& dest);

    //         /**
    //          * manhattan distance between two points(considering only adjacent traversal)
    //          * @param src <int> index of 2d map
    //          * @param dest <int> index of 2d map
    //          * @returns <double> manhattan distance between two points
    //          */
    //         double manhattanDist(const int& src, const int& dest);

    //         /**
    //          * distance based heuristic calculation
    //          * @param src <int> index of 2d map
    //          * @param dest <int> index of 2d map
    //          * @returns <double> distance heuristic between two points
    //          */
    //         double heuristic(const int& src, const int& dest);

    //         /**
    //          * Finds neighbouring nodes of current cell
    //          * @param index <int> index of cell on 2d map
    //          * @returns Integer vector of neighbouring nodes
    //          */
    //         vector<int> findNeighbours(int index);

    //         /**
    //          * Retrace the path based on parent of each node from destination to start
    //          * @param matrix vector of Node containing info about parents of each cell
    //          * @param dest <int> destination index
    //          * @returns path vector of integer indexes from dest to start
    //          */
    //         vector<int> findpath(const vector<Node>& matrix, const int& dest);

    //         /**
    //          * Summation of cost of adjacent cells using a nxn kernel
    //          * @param index <int> index of cell on 2d map
    //          * @param ksize <int> size of kernel
    //          * @returns <double> summation of cost
    //          */
    //         double kernel_cost(int index, int ksize);

    //         /**
    //          * Average cost of adjacent cells using a nxn kernel
    //          * @param index <int> index of cell on 2d map
    //          * @param ksize <int> size of kernel
    //          * @returns <double> average cost
    //          */
    //         double avg_kernel_cost(int index, int ksize);

    //         /**
    //          * Check whether cell is inside the map
    //          * @param row <int> row
    //          * @param col <int> col
    //          * @returns <bool> true if inside
    //          */
    //         bool isInside(const int& row, const int& col);
            
    // };

    // /**
    //  * Hybrid Astar for parking
    //  * @param .. refer PlannerAlgorithm class for more parameters
    //  * @param diss <double> drive distance of neighbour to be travelled from current node
    //  * @param dd[double] array of drive distances to be explored as neighbours 
    //  * @param steer_angles[double] array of steering angles to be used for neighbour exploration
    //  * @param orientationnRes <int> Resolution of 3rd dimension(angles/orientation) of map. 360/orientationRes = no. of angles to be considered
    //  * @param dest_close_x <int> error leniency in x
    //  * @param dest_close_y <int> error leniency in y
    //  * @param dest_close_theta <int> error linency in theta 
    //  */
    // class HAstar_park : public PlannerAlgorithm{ 
    //     public:
    //         double diss = sqrt(2.0) + 0.01;
    //         // double dd[1] = {diss}; //small dist which should be greater than diagonal of unit cell of map so that neighbours do not fall in the same cell again
    //         // double steer_angles[3] = {-M_PI/4,0,M_PI/4}; //angles at which car can steer
    //         double dd[2] = {diss,-diss}; //small dist which should be greater than diagonal of unit cell of map so that neighbours do not fall in the same cell again
    //         double steer_angles[7] = {-M_PI/3,-M_PI/4,-M_PI/6,0,M_PI/6,M_PI/4,M_PI/3}; //angles at which car can steer
    //         int L = 10; // distance between front and rear axle of the car

    //         int orientationRes = 5; //orientation resolution
            
    //         //closeness parameters
    //         int dest_close_x = 0;
    //         int dest_close_y = 0;
    //         int dest_close_theta = 10;

    //         HAstar_park(int w,int h,int len,costmap_2d::Costmap2D* costmap)
    //             : PlannerAlgorithm(w,h,len,costmap)
    //         {}

    //         /**
    //          * search path from given start and destination
    //          * @param src <Index> position and orientation of start cell
    //          * @param dest <Index> position and orientation of goal cell
    //          * @returns path vector <Index> (pos & orientation) 
    //          */
    //         vector<Index> search_path(const Index& src, const Index& dest);

    //         /**
    //          * finding neighbours at drivable distances from current cell
    //          * @param index <Index> position and orientation of current cell
    //          * @returns vector <Index> (pos & orientation) of neighbours
    //          */
    //         vector<Index> findNeighbours(const Index& index);

    //         /**
    //          * round floating values of x&y to integer
    //          * @param x <double> x/y
    //          * @returns integer x/y
    //          */
    //         int get_int_pos(const double& x);

    //         /**
    //          * check if destination is reached based on closeness parameters
    //          * @param neighbour <Index> position and orientation of neighbour cell
    //          * @param dest <Index> position and orientation of destination cell
    //          * @returns <bool> true if neighbour cell is close to destination
    //          */
    //         bool dest_reached(const Index& neighbour, const Index& dest);

    //         /**
    //          * retrace the path from goal to start using parents of each cell
    //          * @param matrix 2d matrix of type <Node_park> containing parent of each cell and f&g costs
    //          * @param dest <Index> position and orientation of destination cell
    //          * @returns path vector <Index> (position and orientaiton) from goal to start
    //          */
    //         vector<Index> findpath(const vector<vector<Node_park>>& matrix, const Index& dest);
    // };

    // /**
    //  * Hybrid Astar for local planning. In progress. 
    //  * The difference from HAstar_park is that we are not treating angle as a part of the map as another dimension, but just storing it as a cell value.
    //  * @param .. refer HAstar_park & PlannerAlgorithm class for more parameters. Below are parameters from HAstar_park
    //  * @param diss <double> drive distance of neighbour to be travelled from current node
    //  * @param dd[double] array of drive distances to be explored as neighbours 
    //  * @param steer_angles[double] array of steering angles to be used for neighbour exploration
    //  * @param orientationnRes <int> Resolution of 3rd dimension(angles/orientation) of map. 360/orientationRes = no. of angles to be considered
    //  * @param dest_close_x <int> error leniency in x
    //  * @param dest_close_y <int> error leniency in y
    //  * @param dest_close_theta <int> error linency in theta 
    //  */
    // class HAstar_local : public HAstar_park{ 
    //     public:
    //         HAstar_local(int w,int h,int len,costmap_2d::Costmap2D* costmap)
    //             : HAstar_park(w,h,len,costmap)
    //         {}

    //         /**
    //          * search path from given start and destination
    //          * @param src <Index> position and orientation of start cell
    //          * @param dest <Index> position and orientation of goal cell
    //          * @returns path vector <Index> (pos & orientation) 
    //          */
    //         vector<Index> search_path(const Index& src, const Index& dest);

    //         /**
    //          * retrace the path from goal to start using parents of each cell
    //          * @param matrix 2d matrix of type <Node_local> containing parent of each cell and f&g costs
    //          * @param dest <Index> position and orientation of destination cell
    //          * @returns path vector <Index> (position and orientaiton) from goal to start
    //          */
    //         vector<Index> findpath(const vector<Node_local>& matrix, const Index& dest);
    // };

    // /**
    //  * Ignore for now
    //  */
    // class Astar_smooth : public PlannerAlgorithm{
    //     public:
    //         Astar_smooth(int w,int h,int len,costmap_2d::Costmap2D* costmap)
    //             : PlannerAlgorithm(w,h,len,costmap)
    //         {}

    //         vector<Index> search_path(const Index& start_pt, const int& dest,const double& accn,vector<double> initial_vel,const int& time_frame,const double& d_robot);
    //         vector<Neighbour_smooth> findNeighbours(const int& index,const int& angle,const double& accn,const double& U_l,const double& U_r,const int& time_frame,const double& d_robot);
    //         vector<Index> findpath(const vector<vector<Node_big>>& matrix, const int& dest);
            
    // };

    // /**
    //  * Astar path planning algorithm
    //  * @param .. refer PlannerAlgorithm class for more parameters
    //  */
    // class Astar : public PlannerAlgorithm{
    //     public:
    //         Astar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
    //             : PlannerAlgorithm(w,h,len,costmap)
    //         {}

    //         /**
    //          * search path from given start and destination
    //          * @param src <int> index of cell
    //          * @param dest <int> index of cell
    //          * @returns path vector <int> indexes  
    //          */
    //         vector<int> search_path(const int& src, const int& dest);
            
    // };

    // /**
    //  * Relaxed A* path planning algorithm. Faster than A* but doesn't guarantee shortest path
    //  * @param .. refer PlannerAlgorithm class for more parameters
    //  */
    // class RAstar : public PlannerAlgorithm{
    //     public:
    //         RAstar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
    //             : PlannerAlgorithm(w,h,len,costmap)
    //         {}

    //         /**
    //          * search path from given start and destination
    //          * @param src <int> index of cell
    //          * @param dest <int> index of cell
    //          * @returns path vector <int> indexes  
    //          */
    //         vector<int> search_path(const int& src, const int& dest);

    //         /**
    //          * retrace path from destination to start based on gscores
    //          * @param src <int> index of cell
    //          * @param dest <int> index of cell
    //          * @returns path vector <int> indexes  
    //          */
    //         vector<int> tracePath(const int& src,const int& dest,const vector<double>& g_score);

    // };

    // /**
    //  * Theta A* path planning algorithm. Any angle planner
    //  * @param .. refer PlannerAlgorithm class for more parameters
    //  */
    // class Tstar : public PlannerAlgorithm{
    //     public:
    //         Tstar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
    //             : PlannerAlgorithm(w,h,len,costmap)
    //         {}

    //         /**
    //          * search path from given start and destination
    //          * @param src <int> index of cell
    //          * @param dest <int> index of cell
    //          * @returns path vector <int> indexes  
    //          */
    //         vector<int> search_path(const int& src, const int& dest);

    //         /**
    //          * check if line of sight exists between points s and s1
    //          * @param s <int> index of cell
    //          * @param s1 <int> index of cell
    //          * @returns 0 for no line of sight 
    //          */
    //         double line_of_sight(const int& s, const int& s1);
            
    // };




//// Global functions in globalplanner namespace:

/**
 * Convert deg to rads
 * @param angle <double>
 * @return angle <double> in rads
 */
double to_rads(const double& angle){
    return (angle*M_PI)/180.0;
}

/**
 * Convert rad to deg
 * @param angle <double>
 * @return angle <int> in deg
 */
int to_deg(const double& angle){
    return (angle*180)/M_PI;
}

/**
 * get deg in range [0,360]
 * @param angle <int>
 * @return angle <int>
 */
int correct_deg(int angle){
    int a=angle;
    if(angle>=360)
        a = angle%360;
    else if(angle<0)
        a = 360 + angle;
    return a;
}

/**
 * convert deg to rads in range [-pi,pi]
 * @param angle <int>
 * @return angle <double>
 */
double deg_to_rads_inrange(int angle){
    int a = correct_deg(angle);
    if(a>=180)
        return to_rads(a-360);
    else
        return to_rads(a);
}

/**
 * round deg based on resolution. say nearest 5s or 10s
 * @param angle <int>
 * @param res <int> resolution
 * @return angle <int> 
 */
int rounded_deg(int angle, int res){
   int rem = angle%res;
   if(rem>=res/2.0) return correct_deg(angle - rem + res);
   else return (angle - rem); 
}

/**
 * Ignore for now
 */
vector<double> find_vel(const int& time_frame,const float& d,const int& x, const int& y, const int& angle, const int& x_prev,const int& y_prev, int angle_prev){
    vector<double> vel;
    double theta = to_rads(angle);
    double theta_prev = to_rads(angle_prev);
    double w = (theta-theta_prev)/time_frame;
    double eqn2 = d*w; // Vr-Vl
    double vx = ((x-x_prev)*w)/sin(w*time_frame);
    double vy = ((y-y_prev)*w)/(1-cos(w*time_frame));
    double eqn1 = (vx+vy); //just to take avg to reduce diff
    double Vr = (eqn1 + eqn2)/2;
    double Vl = (eqn1 - eqn2)/2;
    vel.clear();
    vel.push_back(Vl);
    vel.push_back(Vr);
    return vel;
}

};
#endif
