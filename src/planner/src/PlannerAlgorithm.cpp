#include "../include/PlannerAlgorithm.h"

namespace globalplanner{

//// PlannerAlgorithm methods:

vector<int> PlannerAlgorithm::findpath(const vector<Node>& matrix, const int& dest){
    // cout<<"the path:"<<endl;
    //ROS_INFO("checkpoint findpath 1");
    vector<int> path;
    
    cout<<"findpath pathlen "<<matrix[dest].g<<endl;
    int next_node = matrix[dest].parent;
    path.push_back(dest);
    do{
        path.push_back(next_node);
        next_node = matrix[next_node].parent;
    }while(matrix[next_node].parent!=next_node);

    path.push_back(next_node);
    
    // for(auto i=path.rbegin();i!=path.rend();++i){
    //     cout<<"->"<<(*i)/width<<","<<(*i)%width;

    return path;
}

vector<int> PlannerAlgorithm::findNeighbours(int index){
    int row = index/width;
    int col = index%width;
    vector<int> neighbours;

    for(int add_x=-1; add_x<=1;add_x++)
    {
        for(int add_y=-1; add_y<=1;add_y++)
        {	//ROS_INFO("checkpoint astar while 2");
            if(!(isInside(row+add_y,col+add_x) &&(!(add_x==0 && add_y==0)))) continue;
            //if(!(add_x*add_y == 0)) continue; // if robot can only tranverse in 4 directions 
            int n = index + add_y*width + add_x;
            if(isNotBlocked(n)) neighbours.push_back(n);
        }
    }
    return neighbours;
}

bool PlannerAlgorithm::isInside(const int& row, const int& col){ //input in (i,j) or (row,col)
    return (col >= 0)&&(col < width)&&(row >=0)&&(row < height);
}

double PlannerAlgorithm::distance(const int& src, const int& dest){
    return sqrt(pow((src/width)-(dest/width),2.0) + pow((src%width) - (dest%width),2.0));
}

double PlannerAlgorithm::distance_squared(const int& src, const int& dest){ // reduce computation
    return (pow((src/width)-(dest/width),2.0) + pow((src%width) - (dest%width),2.0));
}

double PlannerAlgorithm::diagonalDist(const int& src, const int& dest){
    int dx = abs(src%width - dest%width);
    int dy = abs(src/width - dest/width);
    double D = 1.0, D2 = 1.4142; // D and D2 straight and diagonal dist. configure to modify heuristc 
    return (D * (dx + dy) + (D2 - 2 * D) * min(dx, dy));
}

double PlannerAlgorithm::manhattanDist(const int& src, const int& dest){
    int dx = abs(src%width - dest%width);
    int dy = abs(src%width - dest%width);
    int D = 1;
    return D * (dx+dy);
}

double PlannerAlgorithm::heuristic(const int& src, const int& dest){
    return distance(src,dest);
}

bool PlannerAlgorithm::isValid(const int& p){
    //return (p.x>=0) && (p.x<row) && (p.y>=0) && (p.y<col);
    return true;
}

bool PlannerAlgorithm::isNotBlocked(const int& p){
    //for binary
    //ROS_INFO("isNotBlocked");
    return isValid(p) && (253>costmap_->getCost(p%width,p/width));
    //for costmap
    //return isValid(map,p);
    //return true; // for the time being. we need both x&y for this
}

double PlannerAlgorithm::kernel_cost(int index, int ksize){
    int k = (ksize-1)/2;
    double cost = 0;
    int row = index/width, col = index%width;
    for(int i = row-k; i<=row+k;i++){
        for(int j=col-k;j<=col+k;j++){
            if(ksize==1 || isInside(i,j))
                cost += costmap_->getCost(j,i); //pass in (x,y format). 
        }
    }
    return cost;
}

double PlannerAlgorithm::avg_kernel_cost(int index, int ksize){
    double cost = kernel_cost(index,ksize);
    return cost/(ksize*ksize);
}

};
