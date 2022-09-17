#include "global_planner.h"
#include "PlannerAlgorithm.h"

namespace globalplanner{
    /**
     * Astar path planning algorithm
     * @param .. refer PlannerAlgorithm class for more parameters
     */
    class Astar : public PlannerAlgorithm{
        public:
            Astar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            /**
             * search path from given start and destination
             * @param src <int> index of cell
             * @param dest <int> index of cell
             * @returns path vector <int> indexes  
             */
            vector<int> search_path(const int& src, const int& dest);
            
    };

};