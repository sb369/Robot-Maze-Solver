#include "global_planner.h"
#include "PlannerAlgorithm.h"

namespace globalplanner{

    /**
     * Hybrid Astar for parking
     * @param .. refer PlannerAlgorithm class for more parameters
     * @param diss <double> drive distance of neighbour to be travelled from current node
     * @param dd[double] array of drive distances to be explored as neighbours 
     * @param steer_angles[double] array of steering angles to be used for neighbour exploration
     * @param orientationnRes <int> Resolution of 3rd dimension(angles/orientation) of map. 360/orientationRes = no. of angles to be considered
     * @param dest_close_x <int> error leniency in x
     * @param dest_close_y <int> error leniency in y
     * @param dest_close_theta <int> error linency in theta 
     */
    class HAstar_park : public PlannerAlgorithm{ 
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

            HAstar_park(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            /**
             * search path from given start and destination
             * @param src <Index> position and orientation of start cell
             * @param dest <Index> position and orientation of goal cell
             * @returns path vector <Index_smoothPath> (pos & orientation) 
             */
            vector<Index_smoothPath> search_path(const Index& src, const Index& dest);

            /**
             * finding neighbours at drivable distances from current cell
             * @param index <ActualPos> position and orientation of current cell
             * @returns vector <Index_smoothPath> (pos & orientation) of neighbours
             */
            vector<Index_smoothPath> findNeighbours(const ActualPos& index);

            /**
             * round floating values of x&y to integer
             * @param x <double> x/y
             * @returns integer x/y
             */
            int get_int_pos(const double& x);

            /**
             * check if destination is reached based on closeness parameters
             * @param neighbour <Index> position and orientation of neighbour cell
             * @param dest <Index> position and orientation of destination cell
             * @returns <bool> true if neighbour cell is close to destination
             */
            bool dest_reached(const Index& neighbour, const Index& dest);

            /**
             * retrace the path from goal to start using parents of each cell
             * @param matrix 2d matrix of type <Node_park> containing parent of each cell and f&g costs
             * @param dest <Index> position and orientation of destination cell
             * @returns path vector <Index> (position and orientaiton) from goal to start
             */
            vector<Index_smoothPath> findpath(const vector<vector<Node_park_smoothPath>>& matrix, const Index& dest);
    };
};