#include "Tstar.h"

namespace globalplanner{

///// Tstar:

vector<int> Tstar::search_path(const int& src, const int& dest){
    //auto start_clock1 = std::chrono::steady_clock::now();

    vector<int> path;
    ROS_INFO("checkpoint astar 1");
    if (!isValid(src)){
        cout<<"source is invalid"<<endl;
        return path;
    }
    if (!isValid(dest)){
        cout<<"dest is invalid"<<endl;
        return path;
    }
    if (!isNotBlocked(dest)){
        cout<<"dest blocked"<<endl;
        return path;
    }
    if (src==dest){
        cout<<"source and dest sameee"<<endl;
        return path;
    }

    //bool visited[maplen]; //already visited/explored cells
    //memset(visited,false,sizeof(visited));
    vector<bool> visited(maplen,false);
    ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<Node> matrix(maplen);
    int i = src;
    matrix[i].f = 0.0;
    matrix[i].g = 0.0;
    //matrix[i].h = 0.0;
    matrix[i].parent = i;
    ROS_INFO("checkpoint astar 3");
    //passing (f,x,y) in priority queue. min_f at top
    //toExplore is to be explored nodes/cells
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> toExplore;
    toExplore.emplace(0.0,i);
    ROS_INFO("checkpoint astar 4");

    int early_successor = -1;
    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        if(early_successor != -1) i = early_successor;
        else{
            const Pair& p = toExplore.top();
            i = get<1>(p);
            toExplore.pop();
        }
	    //auto start_clockpq = std::chrono::steady_clock::now();
	    //auto diff_timepq = start_clockpq - end_clockpq;
        //cout << std::chrono::duration <double, milli> (diff_timepq).count() << " ms for pq" << endl;

        if (visited[i] == true) continue; //to remove duplicate longer nodes
        visited[i] = true;
        ROS_INFO("checkpoint astar while 1");
	    // if(matrix[i].g>30000){ // won't generate full path but small goal faster
	    //     path = findpath(matrix,i);
	    //     ROS_INFO("not finding full path. uncomment");
	    // return path;
	    // }
	ROS_INFO("checkpoint astar while 2");
        early_successor = -1;
        double lowest_neighbour = get<0>(toExplore.top());

        vector<int> neighbours = findNeighbours(i);
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour = *loopvar;
            if(isNotBlocked(neighbour) && !visited[neighbour])
            {   ROS_INFO("checkpoint astar while 3");
                if(neighbour==dest)
                {	ROS_INFO("calling findpath");
                    matrix[neighbour].parent = i;
		    matrix[neighbour].g = matrix[i].g + heuristic(i,neighbour);
		    //cout<<"neighbours g = "<< matrix[i].g<<endl;
                    // cout<<"dest found"<<endl;
                    path.clear();

		            //auto end_clock1 = std::chrono::steady_clock::now();
		            //auto diff_time = end_clock1 - start_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for astar" << endl;

                    path = findpath(matrix,dest);

		            //start_clock1 = std::chrono::steady_clock::now();
		            //diff_time = start_clock1 - end_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for findpath" << endl;

                    return path; 
                }

                else
                {   //cout<<"finding neighboursszzz"<<endl;	
		            double g_new,h_new,f_new,cost_new;
                    int par;
                    cost_new = 2*line_of_sight(matrix[i].parent,neighbour);
                    if(cost_new > 0){
                        g_new = matrix[matrix[i].parent].g + heuristic(matrix[i].parent,neighbour);
                        par = matrix[i].parent;
			            //cost_new += 20*avg_kernel_cost(neighbour,5);
                    }
		            //cout<<"lineofsight ok"<<endl;
                    else{
                        g_new = matrix[i].g + heuristic(i,neighbour);
                        par = i;
                        //cost_new = 20*avg_kernel_cost(neighbour,5);
                    }
 		            cost_new = 3*avg_kernel_cost(neighbour,3);
                    h_new = heuristic(neighbour,dest);
                    f_new = g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + cost_new;

                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix[neighbour].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix[neighbour].g = g_new;
                        //matrix[neighbour].h = h_new;
                        matrix[neighbour].f = f_new;
                        matrix[neighbour].parent = par;

			            if(lowest_neighbour > f_new){
                            if(lowest_neighbour = get<0>(toExplore.top())){
                                lowest_neighbour = matrix[neighbour].f;
                                early_successor = neighbour;
				//ROS_INFO("an early successor");
                                continue;
                            }
                            else{
				//ROS_INFO("switching early successor");
                                lowest_neighbour = matrix[neighbour].f;
                                int temp = early_successor;
                                early_successor = neighbour;
                                neighbour = temp;
                            }
                        }

                        toExplore.emplace(matrix[neighbour].f,neighbour);
                    }
                }
                    
            }
        }
        
    }
    cout<<"dest not found"<<endl;
    

}



double Tstar::line_of_sight(const int& s, const int& s1){
    int x0 = s%width;
    int x1 = s1%width;
    int y0 = s/width;
    int y1 = s1/width;

    double cost = 0;
    int dy = y1-y0 , dx = x1-x0, f,sy,sx;
    if(dy<0){
        dy = -dy;
        sy = -1;
    }
    else sy = 1;

    if(dx<0){
        dx = -dx;
        sx = -1;
    }
    else sx = 1;

    if(dx>=dy){
        while(x0!=x1){
            f += dy;
            if(f>=dx && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2))){
                if(costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253)
                    return 0;
                //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
                y0 += sy;
                f = f-dx;
            }

            if(f!=0 && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2)) && (costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253))
                return 0;
            //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));

            if(isInside(y0,x0 + ((sx-1)/2)) && isInside(y0-1,x0 + ((sx-1),2)))   
                if(dy==0 && (costmap_->getCost(x0 + ((sx-1)/2), y0) >= 253) && (costmap_->getCost(x0 + ((sx-1),2), y0-1) >= 253))
                    return 0;
            //cost = cost + costmap_->getCost(x0 + ((sx-1)/2), y0) + costmap_->getCost(x0 + ((sx-1),2), y0-1);
            x0 = x0 + sx;

        }
    }
    else{
        while(y0!=y1){
            f += dx;
            if(f>=dy  && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2))){
                if(costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253)
                    return 0;
                //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
                x0 += sx;
                f = f - dy;
            }
            if(f!=0 && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2)) && (costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253))
                return 0;
            //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
            if(isInside(y0+((sy-1)/2),x0) && isInside(y0 + ((sy-1)/2),x0-1))
                if(dx==0 && (costmap_->getCost(x0, y0+((sy-1)/2)) >= 253) && (costmap_->getCost(x0-1 , y0 + ((sy-1)/2)) >= 253))
                    return 0;
            //cost = cost + costmap_->getCost(x0, y0+((sy-1)/2)) + costmap_->getCost(x0-1 , y0 + ((sy-1)/2));
            y0 += sy;
        }
    }
    return cost+1;
}


};