#include "global_planner.h"
#include "PlannerAlgorithm.h"

namespace globalplanner{
    /**
     * Relaxed A* path planning algorithm. Faster than A* but doesn't guarantee shortest path
     * @param .. refer PlannerAlgorithm class for more parameters
     */
    class RAstar : public PlannerAlgorithm{
        public:
            RAstar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            /**
             * search path from given start and destination
             * @param src <int> index of cell
             * @param dest <int> index of cell
             * @returns path vector <int> indexes  
             */
            vector<int> search_path(const int& src, const int& dest);

            /**
             * retrace path from destination to start based on gscores
             * @param src <int> index of cell
             * @param dest <int> index of cell
             * @returns path vector <int> indexes  
             */
            vector<int> tracePath(const int& src,const int& dest,const vector<double>& g_score);

    };

};
