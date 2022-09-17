vector<Index> HAstar_park::search_path(const Index& src, const Index& dest){
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
    vector<vector<bool>> visited3d(maplen,vector<bool>(360,false));
    //ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<vector<Node_park>> matrix3d(maplen,vector<Node_park>(360));
    int i = src.pos;
    int theta = src.orientation;
    matrix3d[i][theta].f = 0.0;
    matrix3d[i][theta].g = 0.0;
    //matrix[i].h = 0.0;
    matrix3d[i][theta].parent.pos = i;
    matrix3d[i][theta].parent.orientation = theta;

    std::priority_queue<Tuple, std::vector<Tuple>, std::greater<Tuple>> toExplore;
    toExplore.emplace(0.0,i,theta);
    //ROS_INFO("checkpoint astar 4");

    //int early_successor = -1;
    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        //if(early_successor > -1) i = early_successor;
    
        const Tuple& p = toExplore.top();
        i = get<1>(p);
        theta = get<2>(p);
        toExplore.pop();
        
	    //auto start_clockpq = std::chrono::steady_clock::now();
	    //auto diff_timepq = start_clockpq - end_clockpq;
        //cout << std::chrono::duration <double, milli> (diff_timepq).count() << " ms for pq" << endl;

        if (visited3d[i][theta] == true) continue; //to remove duplicate longer nodes
        visited3d[i][theta] = true;
        //ROS_INFO("checkpoint astar while 1");
	    if(matrix3d[i][theta].g>30000){ // won't generate full path but small goal faster
	        path = findpath(matrix3d,dest);
	        ROS_INFO("not finding full path. uncomment");
	    return path;
	    }
	    //ROS_INFO("checkpoint astar while 2");
        //early_successor = -1;
        //double lowest_neighbour = get<0>(toExplore.top());
        vector<Index> neighbours = findNeighbours({i,theta});
        //cout<<"neighbour size: "<<neighbours.size()<<endl;
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour_i = loopvar->pos;
            int neighbour_theta = loopvar->orientation;
            //ROS_INFO("pos %d orient %d  d %f c %f vl %f vr %f",loopvar->pos,loopvar->orientation,loopvar->d,loopvar->c,loopvar->V_l,loopvar->V_r);
            if(isNotBlocked(neighbour_i) && !visited3d[neighbour_i][neighbour_theta])
            {   //ROS_INFO("checkpoint astar while 3");
                 if(dest_reached({neighbour_i,neighbour_theta},dest))
                {	//ROS_INFO("calling findpath");
                    matrix3d[dest.pos][dest.orientation].parent.pos = i;
                    matrix3d[dest.pos][dest.orientation].parent.orientation = theta;
		            matrix3d[dest.pos][dest.orientation].g = matrix3d[i][theta].g + heuristic(i,dest.pos);
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
                    g_new = matrix3d[i][theta].g + heuristic(i,neighbour_i);
                    h_new = heuristic(neighbour_i,dest.pos);

                    // SELECT APPROPRIATE F COST
                    f_new = g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + avg_kernel_cost(neighbour_i,1);
                    f_new = h_new + avg_kernel_cost(neighbour_i,1); 
                    
                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix3d[neighbour_i][neighbour_theta].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix3d[neighbour_i][neighbour_theta].g = g_new;
                        //matrix[neighbour].h = h_new;  //removed from Node definition
                        matrix3d[neighbour_i][neighbour_theta].f = f_new;
                        matrix3d[neighbour_i][neighbour_theta].parent.pos = i;
                        matrix3d[neighbour_i][neighbour_theta].parent.orientation = theta;
                        toExplore.emplace(matrix3d[neighbour_i][neighbour_theta].f,neighbour_i,neighbour_theta);
                    }
                } 
            }
        }
    }
    cout<<"dest not found"<<endl;
}

vector<Index> HAstar_park::findNeighbours(const Index& index){
    int current_x = index.pos%width;
    int current_y = index.pos/width;
    vector<Index> neighbours;

    double theta = to_rads(index.orientation);
    for(int counter_i=0; counter_i<(sizeof(dd)/sizeof(dd[0]));counter_i++){
        for(int counter_j=0; counter_j<(sizeof(steer_angles)/sizeof(steer_angles[0])); counter_j++){
            double alpha = steer_angles[counter_j];
            double beta = (dd[counter_i] / L) * tan(alpha);

            //calculate new pos
            double next_x, next_y;
            int neighbour_x, neighbour_y;
            if(abs(beta)<0.00001f){
                next_x = current_x - dd[counter_i] * cos(theta);
                next_y = current_y + dd[counter_i] * sin(theta);
            }

            else{
                double R = dd[counter_i] / beta;

                double cx = current_x + sin(theta) * R;
                double cy = current_y + cos(theta) * R;

                next_x = cx + sin(theta + beta) * R;
                next_y = cy - cos(theta + beta) * R;
            }

            neighbour_x = get_int_pos(next_x);
            neighbour_y = get_int_pos(next_y);

            int new_theta = int(correct_deg(to_deg(theta + beta)));
            
            neighbours.push_back({(neighbour_y/width) + (neighbour_x%width),new_theta});
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
    if(((neighbour.pos%width - dest.pos%width) <= dest_close_x) && ((neighbour.pos/width - dest.pos/width) <= dest_close_y) && ((neighbour.orientation - dest.orientation) <= dest_close_theta)){
    reached = true;
    }
    return reached;
}

vector<Index> HAstar_park::findpath(const vector<vector<Node_park>>& matrix, const Index& dest){
    // cout<<"the path:"<<endl;
    //ROS_INFO("checkpoint findpath 1");
    vector<Index> path;
    
    cout<<"findpath pathlen "<<matrix[dest.pos][dest.orientation].g<<endl;
    Index next_node;
    next_node.pos = dest.pos;
    next_node.orientation = dest.orientation;
    path.push_back(next_node);
    next_node.pos = matrix[dest.pos][dest.orientation].parent.pos;
    next_node.orientation = matrix[dest.pos][dest.orientation].parent.orientation;
    do{
        path.push_back(next_node);
        int i = matrix[next_node.pos][next_node.orientation].parent.pos;
        int theta = matrix[next_node.pos][next_node.orientation].parent.orientation;
        next_node.pos = i;
        next_node.orientation = theta;
    }while(matrix[next_node.pos][next_node.orientation].parent.pos!=next_node.pos && matrix[next_node.pos][next_node.orientation].parent.orientation!=next_node.orientation);

    path.push_back(next_node);
    
    // for(auto i=path.rbegin();i!=path.rend();++i){
    //     cout<<"->"<<(*i)/width<<","<<(*i)%width;

    return path;
}


///// HAstar_local methods:

vector<Index> HAstar_local::search_path(const Index& src, const& Index& dest);{
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

        vector<Index> neighbours = findNeighbours({i,theta});

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

vector<Index> HAstar_local::findpath(const vector<vector<Node_local>>& matrix, const Index& dest){
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
    
    // for(auto i=path.rbegin();i!=path.rend();++i){
    //     cout<<"->"<<(*i)/width<<","<<(*i)%width;

    return path;
}