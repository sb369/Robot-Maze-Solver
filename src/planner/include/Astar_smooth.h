#include "global_planner.h"
#include "PlannerAlgorithm.h"

namespace globalplanner{
    /**
     * Ignore for now
     */
    class Astar_smooth : public PlannerAlgorithm{
        public:
            Astar_smooth(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<Index> search_path(const Index& start_pt, const int& dest,const double& accn,vector<double> initial_vel,const int& time_frame,const double& d_robot);
            vector<Neighbour_smooth> findNeighbours(const int& index,const int& angle,const double& accn,const double& U_l,const double& U_r,const int& time_frame,const double& d_robot);
            vector<Index> findpath(const vector<vector<Node_big>>& matrix, const int& dest);
            
    };

};