#include "../include/HAstar_park.h"

namespace globalplanner{

////// HAstar_park methods:

vector<Index_smoothPath> HAstar_park::search_path(const Index& src, const Index& dest){
    //auto start_clock1 = std::chrono::steady_clock::now();
    // cout<<endl<<"start cell received by algo:"<<src.pos%width<<","<<src.pos/width<<endl;
    vector<Index_smoothPath> path;
    
    // ROS_INFO("checkpoint astar 1");
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
    int total_orientations = int(360/orientationRes);
    vector<vector<bool>> visited3d(maplen,vector<bool>(total_orientations,false));
    //ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<vector<Node_park_smoothPath>> matrix3d(maplen,vector<Node_park_smoothPath>(total_orientations));
    int i = src.pos;
    int theta = rounded_deg(src.orientation,orientationRes);
    matrix3d[i][int(theta/orientationRes)].f = 0.0;
    matrix3d[i][int(theta/orientationRes)].g = 0.0;
    //matrix[i].h = 0.0;
    matrix3d[i][int(theta/orientationRes)].parent.pos = i;
    matrix3d[i][int(theta/orientationRes)].parent.orientation = theta;
    matrix3d[i][int(theta/orientationRes)].pose = {i%width,i/width,src.orientation};

    std::priority_queue<Tuple, std::vector<Tuple>, std::greater<Tuple>> toExplore;
    toExplore.emplace(0.0,i,theta);
    //ROS_INFO("checkpoint astar 4");

    //int early_successor = -1;
    int explored_cell_count = 0;
    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        //if(early_successor > -1) i = early_successor;
        explored_cell_count++;
        const Tuple& p = toExplore.top();
        i = get<1>(p);
        theta = get<2>(p);
        toExplore.pop();
        // cout<<endl<<"current cell:"<<i%width<<",\t"<<i/width<<",\t"<<theta<<endl;

	    //auto start_clockpq = std::chrono::steady_clock::now();
	    //auto diff_timepq = start_clockpq - end_clockpq;
        //cout << std::chrono::duration <double, milli> (diff_timepq).count() << " ms for pq" << endl;

        if (visited3d[i][int(theta/orientationRes)] == true) continue; //to remove duplicate longer nodes
        visited3d[i][int(theta/orientationRes)] = true;
        //ROS_INFO("checkpoint astar while 1");
	    // if(matrix3d[i][theta].g>30000){ // won't generate full path but small goal faster
	    //     path = findpath(matrix3d,dest);
	    //     ROS_INFO("not finding full path. uncomment");
	    // return path;
	    // }
	    //ROS_INFO("checkpoint astar while 2");
        //early_successor = -1;
        //double lowest_neighbour = get<0>(toExplore.top());
        vector<Index_smoothPath> neighbours = findNeighbours(matrix3d[i][int(theta/orientationRes)].pose);
        //cout<<"neighbour size: "<<neighbours.size()<<endl;
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour_i = loopvar->pos;
            int neighbour_theta = rounded_deg(loopvar->orientation,orientationRes);
            double neighbour_dd = loopvar->dd;
            double neighbour_steer_angle = loopvar->steer_angle;
            ActualPos neighbour_actual = loopvar->pose;
            //ROS_INFO("pos %d orient %d  d %f c %f vl %f vr %f",loopvar->pos,loopvar->orientation,loopvar->d,loopvar->c,loopvar->V_l,loopvar->V_r);
            if(isNotBlocked(neighbour_i) && !visited3d[neighbour_i][int(neighbour_theta/orientationRes)])
            {   //ROS_INFO("checkpoint astar while 3");
                 if(dest_reached({neighbour_i,neighbour_theta},dest))
                {	//ROS_INFO("calling findpath");
                    matrix3d[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].parent.pos = i;
                    matrix3d[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].parent.orientation = theta;
		            matrix3d[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].g = matrix3d[i][int(theta/orientationRes)].g + heuristic(i,dest.pos);
		           matrix3d[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].dd= neighbour_dd;
                    matrix3d[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].steer_angle = neighbour_steer_angle;
                    matrix3d[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].pose = {dest.pos%width,dest.pos/width,dest.orientation};
                    //cout<<"neighbours g = "<< matrix[i].g<<endl;
                    // cout<<"dest found"<<endl;
                    path.clear();

		            //auto end_clock1 = std::chrono::steady_clock::now();
		            //auto diff_time = end_clock1 - start_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for astar" << endl;

                    path = findpath(matrix3d,dest);

		            //start_clock1 = std::chrono::steady_clock::now();
		            //diff_time = start_clock1 - end_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for findpath" << endl;

                    return path; 
                }

                else
                {	//ROS_INFO("adding neighbours to be explored");
                    double g_new,h_new,f_new;
                    g_new = matrix3d[i][int(theta/orientationRes)].g + heuristic(i,neighbour_i);
                    h_new = heuristic(neighbour_i,dest.pos);

                    // SELECT APPROPRIATE F COST
                    f_new = h_new + avg_kernel_cost(neighbour_i,1); 
                    f_new = 5*g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + 1*avg_kernel_cost(neighbour_i,3) + 5*(neighbour_dd<0? 1 : 0) + abs(neighbour_steer_angle)*5; 
                    
                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].g = g_new;
                        //matrix[neighbour].h = h_new;  //removed from Node definition
                        matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].f = f_new;
                        matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].parent.pos = i;
                        matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].parent.orientation = theta;
                        matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].dd = neighbour_dd;
                        matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].steer_angle = neighbour_steer_angle;
                        matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].pose = neighbour_actual;
                        toExplore.emplace(matrix3d[neighbour_i][int(neighbour_theta/orientationRes)].f,neighbour_i,neighbour_theta);
                    }
                } 
            }
        }
    }
    cout<<"dest not found"<<endl;
    cout<<"explored cell count = "<<explored_cell_count<<endl;
}

vector<Index_smoothPath> HAstar_park::findNeighbours(const ActualPos& index){
    double current_x = index.x;
    double current_y = index.y;
    vector<Index_smoothPath> neighbours;

    double theta = index.orientation;
    for(int counter_i=0; counter_i<(sizeof(dd)/sizeof(dd[0]));counter_i++){
        for(int counter_j=0; counter_j<(sizeof(steer_angles)/sizeof(steer_angles[0])); counter_j++){
            double alpha = steer_angles[counter_j];
            double beta = (dd[counter_i] / L) * tan(alpha);

            //calculate new pos
            double next_x, next_y;
            int neighbour_x, neighbour_y;
            if(abs(beta)<0.00001f){
                next_x = current_x + dd[counter_i] * cos(theta);
                next_y = current_y + dd[counter_i] * sin(theta);
            }

            else{
                double R = dd[counter_i] / beta;

                double cx = current_x - sin(theta) * R;
                double cy = current_y + cos(theta) * R;

                next_x = cx + sin(theta + beta) * R;
                next_y = cy - cos(theta + beta) * R;
            }

            neighbour_x = get_int_pos(next_x);
            neighbour_y = get_int_pos(next_y);

            int new_theta = int(correct_deg(to_deg(theta + beta)));
            
            if(isInside(neighbour_y,neighbour_x) && isNotBlocked(neighbour_y*width + neighbour_x) && (!(neighbour_x==round(current_x) && neighbour_y==round(current_y) && rounded_deg(new_theta,orientationRes)==rounded_deg(to_deg(index.orientation),orientationRes)))){
                neighbours.push_back({neighbour_y*width + neighbour_x, new_theta, dd[counter_i], alpha, {next_x,next_y,to_rads(new_theta)} });

                // cout<<endl<<"neighbours added:"<<neighbour_x<<",\t"<<neighbour_y<<",\t"<<new_theta<<endl;
            }
        }
    }
    return neighbours;
}

int HAstar_park::get_int_pos(const double& x){
    double decimal_part = x - int(x);
    int new_x;
    if(decimal_part>=0.5)
    {
        new_x = int(x)+1;
    }
    else{
        new_x = int(x);
    }
    return new_x;
}

bool HAstar_park::dest_reached(const Index& neighbour, const Index& dest){
    bool reached = false;
    if((abs(neighbour.pos%width - dest.pos%width) <= dest_close_x) && (abs(neighbour.pos/width - dest.pos/width) <= dest_close_y) && (abs(neighbour.orientation - dest.orientation) <= dest_close_theta)){
    reached = true;
    }
    return reached;
}

vector<Index_smoothPath> HAstar_park::findpath(const vector<vector<Node_park_smoothPath>>& matrix, const Index& dest){
    // cout<<"the path:"<<endl;
    //ROS_INFO("checkpoint findpath 1");
    vector<Index_smoothPath> path;
    
    cout<<"findpath pathlen "<<matrix[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].g<<endl;
    Index_smoothPath next_node;
    next_node.pos = dest.pos;
    next_node.orientation = dest.orientation;
    next_node.dd = matrix[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].dd;
    next_node.steer_angle = matrix[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].steer_angle;
    next_node.pose = matrix[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].pose;
    path.push_back(next_node);
    next_node.pos = matrix[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].parent.pos;
    next_node.orientation = matrix[dest.pos][int(rounded_deg(dest.orientation,orientationRes)/orientationRes)].parent.orientation;
    next_node.dd = matrix[next_node.pos][int(next_node.orientation/orientationRes)].dd;
    next_node.steer_angle = matrix[next_node.pos][int(next_node.orientation/orientationRes)].steer_angle;
    next_node.pose = matrix[next_node.pos][int(next_node.orientation/orientationRes)].pose;


    // cout<<endl<<"i theta";
    do{
        path.push_back(next_node);
        int i = matrix[next_node.pos][int(next_node.orientation/orientationRes)].parent.pos;
        int theta = matrix[next_node.pos][int(next_node.orientation/orientationRes)].parent.orientation;
        next_node.pos = i;
        next_node.orientation = theta;
        next_node.dd = matrix[next_node.pos][int(next_node.orientation/orientationRes)].dd;
        next_node.steer_angle = matrix[next_node.pos][int(next_node.orientation/orientationRes)].steer_angle;
        next_node.pose = matrix[next_node.pos][int(next_node.orientation/orientationRes)].pose;
        // cout<<"->"<<i%width<<","<<i/width<<","<<theta<<"\t parent:"<<matrix[i][theta].parent.pos%width<<","<<matrix[i][theta].parent.pos/width<<","<<matrix[i][theta].parent.orientation<<endl;
    }while(matrix[next_node.pos][int(next_node.orientation/orientationRes)].parent.pos!=next_node.pos || matrix[next_node.pos][int(next_node.orientation/orientationRes)].parent.orientation!=next_node.orientation);

    path.push_back(next_node);
    // cout<<endl;
    for(auto i=path.rbegin();i!=path.rend();++i){
        // cout<<"->    "<<(i->pos)%width<<","<<(i->pos)/width<<"orient: "<<i->orientation<<" d: "<<i->dd<<" steer: "<<i->steer_angle<<endl;
    }
    // cout<<endl;
    return path;
}

};
