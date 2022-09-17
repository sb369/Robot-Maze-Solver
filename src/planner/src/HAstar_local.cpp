#include "HAstar_local.h"

namespace globalplanner{
///// HAstar_local methods:

vector<Index> HAstar_local::search_path(const Index& src, const Index& dest){
        //auto start_clock1 = std::chrono::steady_clock::now();

    vector<Index> path;
    
    //ROS_INFO("checkpoint astar 1");
    if (!isValid(src.pos)){
        cout<<"source is invalid"<<endl;
        return path;
    }
    if (!isValid(dest.pos)){
        cout<<"dest is invalid"<<endl;
        return path;
    }
    if (!isNotBlocked(dest.pos)){
        cout<<"dest blocked"<<endl;
        return path;
    }
    if (src.pos==dest.pos){
        cout<<"source and dest sameee"<<endl;
        return path;
    }

    //bool visited[maplen]; //already visited/explored cells
    //memset(visited,false,sizeof(visited));
    vector<bool> visited(maplen,false);
    //ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<Node_local> matrix(maplen);
    int i = src.pos;
    int theta = src.orientation;
    matrix[i].f = 0.0;
    matrix[i].g = 0.0;
    //matrix[i].h = 0.0;
    matrix[i].parent = i;
    matrix[i].orientation = src.orientation;
    //ROS_INFO("checkpoint astar 3");
    //passing (f,x,y) in priority queue. min_f at top
    //toExplore is to be explored nodes/cells
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> toExplore;
    toExplore.emplace(0.0,i);
    //ROS_INFO("checkpoint astar 4");

    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        const Pair& p = toExplore.top();
        i = get<1>(p);
        theta = matrix[i].orientation;
        toExplore.pop();

        if (visited[i]==true) continue;
        visited[i] = true;

        vector<Index_smoothPath> neighbours = findNeighbours({i%width,i/width,theta});

        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour_i = loopvar->pos;
            int neighbour_theta = loopvar->orientation;
            //ROS_INFO("pos %d orient %d  d %f c %f vl %f vr %f",loopvar->pos,loopvar->orientation,loopvar->d,loopvar->c,loopvar->V_l,loopvar->V_r);
            if(isNotBlocked(neighbour_i) && !visited[neighbour_i])
            {   //ROS_INFO("checkpoint astar while 3");
                 if(dest_reached({neighbour_i,neighbour_theta},dest))
                {	//ROS_INFO("calling findpath");
                    matrix[dest.pos].parent = i;
                    matrix[dest.pos].orientation = theta;
		            matrix[dest.pos].g = matrix[i].g + heuristic(i,dest.pos);
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
                {	//ROS_INFO("adding neighbours to be explored");
                    double g_new,h_new,f_new;
                    g_new = matrix[i].g + heuristic(i,neighbour_i);
                    h_new = heuristic(neighbour_i,dest.pos);

                    // SELECT APPROPRIATE F COST
                    f_new = g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + avg_kernel_cost(neighbour_i,1); 
                    
                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix[neighbour_i].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix[neighbour_i].g = g_new;
                        //matrix[neighbour].h = h_new;  //removed from Node definition
                        matrix[neighbour_i].f = f_new;
                        matrix[neighbour_i].parent = i;
                        matrix[neighbour_i].orientation = neighbour_theta;
                        toExplore.emplace(matrix[neighbour_i].f,neighbour_i);
                    }
                } 
            }
        }
    }
}

vector<Index> HAstar_local::findpath(const vector<Node_local>& matrix, const Index& dest){
    vector<Index> path;

    cout<<"findpath pathlen "<<matrix[dest.pos].g<<endl;
    path.push_back(dest);
    Index next_node;
    next_node.pos = matrix[dest.pos].parent;
    next_node.orientation = matrix[matrix[dest.pos].parent].orientation;
    do{
        path.push_back(next_node);
        int i = matrix[next_node.pos].parent;
        int theta = matrix[i].orientation;
        next_node.pos = i;
        next_node.orientation = theta;
    }while(matrix[next_node.pos].parent!=next_node.pos);

    path.push_back(next_node);
    
    for(auto i=path.rbegin();i!=path.rend();++i){
        cout<<"->"<<(i->pos)%width<<","<<(i->pos)/width;
    }

    return path;
}

};