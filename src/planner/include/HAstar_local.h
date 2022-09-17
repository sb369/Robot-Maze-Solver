#include "global_planner.h"
#include "PlannerAlgorithm.h"
#include "HAstar_park.h"

namespace globalplanner{

    /**
     * Hybrid Astar for local planning. In progress. 
     * The difference from HAstar_park is that we are not treating angle as a part of the map as another dimension, but just storing it as a cell value.
     * @param .. refer HAstar_park & PlannerAlgorithm class for more parameters. Below are parameters from HAstar_park
     * @param diss <double> drive distance of neighbour to be travelled from current node
     * @param dd[double] array of drive distances to be explored as neighbours 
     * @param steer_angles[double] array of steering angles to be used for neighbour exploration
     * @param orientationnRes <int> Resolution of 3rd dimension(angles/orientation) of map. 360/orientationRes = no. of angles to be considered
     * @param dest_close_x <int> error leniency in x
     * @param dest_close_y <int> error leniency in y
     * @param dest_close_theta <int> error linency in theta 
     */
    class HAstar_local : public HAstar_park{ 
        public:
            HAstar_local(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : HAstar_park(w,h,len,costmap)
            {}

            /**
             * search path from given start and destination
             * @param src <Index> position and orientation of start cell
             * @param dest <Index> position and orientation of goal cell
             * @returns path vector <Index> (pos & orientation) 
             */
            vector<Index> search_path(const Index& src, const Index& dest);

            /**
             * retrace the path from goal to start using parents of each cell
             * @param matrix 2d matrix of type <Node_local> containing parent of each cell and f&g costs
             * @param dest <Index> position and orientation of destination cell
             * @returns path vector <Index> (position and orientaiton) from goal to start
             */
            vector<Index> findpath(const vector<Node_local>& matrix, const Index& dest);
    };

};