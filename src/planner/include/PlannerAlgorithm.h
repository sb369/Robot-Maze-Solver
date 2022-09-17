#include "global_planner.h"

#ifndef PLANNER_ALGORITHM_H
#define PLANNER_ALGORITHM_H

namespace globalplanner{

    /**
     * PlannerAlgorithm: A general template class for all planners
     * @param width <int> width of the map
     * @param height <int> height of the map
     * @param maplen <int> size of map
     * @param costmap_ <costmap_2d::Costmap2D> costmap 
     */
    class PlannerAlgorithm{
        public:
            int width, height, maplen;
            costmap_2d::Costmap2D* costmap_;

            PlannerAlgorithm(int w,int h,int len,costmap_2d::Costmap2D* costmap){
                width = w;
                height = h;
                maplen = len;
                costmap_ = costmap;
            }

            /**
             * If cell lies inside map
             * @param p <int> index of 2d map
             */
            bool isValid(const int& p);

            /**
             * Check if the cell is not an obstacle or not
             * @param p <int> index of 2d map
             */
            bool isNotBlocked(const int& p);

            /**
             * Euclidiean distance between two points
             * @param src <int> index of 2d map
             * @param dest <int> index of 2d map
             * @returns <double> euclidean distance between two points
             */
            double distance(const int& src, const int& dest);

            /**
             * square of distance between two points
             * @param src <int> index of 2d map
             * @param dest <int> index of 2d map
             * @returns <double> square of distance between two points
             */
            double distance_squared(const int& src, const int& dest);

            /**
             * distance between two points 
             * 1 for adjacent, 1.4142 for diagonal
             * @param src <int> index of 2d map
             * @param dest <int> index of 2d map
             * @returns <double> distance between two points
             */
	        double diagonalDist(const int& src, const int& dest);

            /**
             * manhattan distance between two points(considering only adjacent traversal)
             * @param src <int> index of 2d map
             * @param dest <int> index of 2d map
             * @returns <double> manhattan distance between two points
             */
            double manhattanDist(const int& src, const int& dest);

            /**
             * distance based heuristic calculation
             * @param src <int> index of 2d map
             * @param dest <int> index of 2d map
             * @returns <double> distance heuristic between two points
             */
            double heuristic(const int& src, const int& dest);

            /**
             * Finds neighbouring nodes of current cell
             * @param index <int> index of cell on 2d map
             * @returns Integer vector of neighbouring nodes
             */
            vector<int> findNeighbours(int index);

            /**
             * Retrace the path based on parent of each node from destination to start
             * @param matrix vector of Node containing info about parents of each cell
             * @param dest <int> destination index
             * @returns path vector of integer indexes from dest to start
             */
            vector<int> findpath(const vector<Node>& matrix, const int& dest);

            /**
             * Summation of cost of adjacent cells using a nxn kernel
             * @param index <int> index of cell on 2d map
             * @param ksize <int> size of kernel
             * @returns <double> summation of cost
             */
            double kernel_cost(int index, int ksize);

            /**
             * Average cost of adjacent cells using a nxn kernel
             * @param index <int> index of cell on 2d map
             * @param ksize <int> size of kernel
             * @returns <double> average cost
             */
            double avg_kernel_cost(int index, int ksize);

            /**
             * Check whether cell is inside the map
             * @param row <int> row
             * @param col <int> col
             * @returns <bool> true if inside
             */
            bool isInside(const int& row, const int& col);
            
    };

};
#endif