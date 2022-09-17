#include "global_planner.h"
#include "PlannerAlgorithm.h"

namespace globalplanner{
    /**
     * Theta A* path planning algorithm. Any angle planner
     * @param .. refer PlannerAlgorithm class for more parameters
     */
    class Tstar : public PlannerAlgorithm{
        public:
            Tstar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
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
             * check if line of sight exists between points s and s1
             * @param s <int> index of cell
             * @param s1 <int> index of cell
             * @returns 0 for no line of sight 
             */
            double line_of_sight(const int& s, const int& s1);
            
    };
};