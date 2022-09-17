#include "Astar_smooth.h"

namespace globalplanner{
///// Astar_smooth methods:

vector<Index> Astar_smooth::search_path(const Index& start_pt, const int& dest,const double& accn,vector<double> initial_vel,const int& time_frame,const double& d_robot){
    //auto start_clock1 = std::chrono::steady_clock::now();

    vector<Index> path;
    int src = start_pt.pos;
    //ROS_INFO("checkpoint astar 1");
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
    vector<vector<bool>> visited3d(maplen,vector<bool>(360,false));
    //ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<vector<Node_big>> matrix3d(maplen,vector<Node_big>(360));
    int i = src;
    int theta = start_pt.orientation;
    matrix3d[i][start_pt.orientation].f = 0.0;
    matrix3d[i][start_pt.orientation].g = 0.0;
    //matrix[i].h = 0.0;
    matrix3d[i][start_pt.orientation].parent.pos = i;
    matrix3d[i][start_pt.orientation].parent.orientation = start_pt.orientation;
    matrix3d[i][theta].vel_l = initial_vel[0];
    matrix3d[i][theta].vel_r = initial_vel[1];
    //ROS_INFO("checkpoint astar 3");
    //passing (f,x,y) in priority queue. min_f at top
    //toExplore is to be explored nodes/cells
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
	    // if(matrix3d[i][theta].g>30000){ // won't generate full path but small goal faster
	    //     path = findpath(matrix3d,i);
	    //     ROS_INFO("not finding full path. uncomment");
	    // return path;
	    // }
	//ROS_INFO("checkpoint astar while 2");
        //early_successor = -1;
        //double lowest_neighbour = get<0>(toExplore.top());

        vector<Neighbour_smooth> neighbours = findNeighbours(i,theta,accn,matrix3d[i][theta].vel_l,matrix3d[i][theta].vel_r,time_frame,d_robot);
        //cout<<"neighbour size: "<<neighbours.size()<<endl;
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour_i = loopvar->pos;
            int neighbour_theta = loopvar->orientation;
            //ROS_INFO("pos %d orient %d  d %f c %f vl %f vr %f",loopvar->pos,loopvar->orientation,loopvar->d,loopvar->c,loopvar->V_l,loopvar->V_r);
            if(isNotBlocked(neighbour_i) && !visited3d[neighbour_i][neighbour_theta])
            {   //ROS_INFO("checkpoint astar while 3");
                if(neighbour_i==dest)
                {	//ROS_INFO("calling findpath");
                    matrix3d[neighbour_i][0].parent.pos = i;
                    matrix3d[neighbour_i][0].parent.orientation = theta;
		            matrix3d[neighbour_i][0].g = matrix3d[i][theta].g + loopvar->d;
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
                    g_new = matrix3d[i][theta].g + loopvar->d; //1 for diagonal?
                    h_new = heuristic(neighbour_i,dest);
                    f_new = loopvar->c + g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + avg_kernel_cost(neighbour_i,1);

                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix3d[neighbour_i][neighbour_theta].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix3d[neighbour_i][neighbour_theta].g = g_new;
                        //matrix[neighbour].h = h_new;  //removed from Node definition
                        matrix3d[neighbour_i][neighbour_theta].f = f_new;
                        matrix3d[neighbour_i][neighbour_theta].parent.pos = i;
                        matrix3d[neighbour_i][neighbour_theta].parent.orientation = theta;
                        matrix3d[neighbour_i][neighbour_theta].vel_l = loopvar->V_l;
                        matrix3d[neighbour_i][neighbour_theta].vel_r = loopvar->V_r;

			            
                        toExplore.emplace(matrix3d[neighbour_i][neighbour_theta].f,neighbour_i,neighbour_theta);
                    }
                }
                    
            }
        }
        
    }
    cout<<"dest not found"<<endl;
    

}

vector<Index> Astar_smooth::findpath(const vector<vector<Node_big>>& matrix, const int& dest){
    // cout<<"the path:"<<endl;
    //ROS_INFO("checkpoint findpath 1");
    vector<Index> path;
    
    cout<<"findpath pathlen "<<matrix[dest][0].g<<endl;
    Index next_node;
    next_node.pos = dest;
    next_node.orientation = 0;
    path.push_back(next_node);
    next_node.pos = matrix[dest][0].parent.pos;
    next_node.orientation = matrix[dest][0].parent.orientation;
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

vector<Neighbour_smooth> Astar_smooth::findNeighbours(const int& index,const int& angle,const double& accn,const double& U_l,const double& U_r,const int& time_frame,const double& d_robot){
    int Yi = index/width;
    int Xi = index%width;
    double theta_i = to_rads(angle);
    vector<Neighbour_smooth> neighbours;
    double vel_change[5][2] = {{-1*accn,accn},{0,accn},{0,0},{accn,0},{accn,-1*accn}};

    for(int counter = 0; counter<5;counter++){
        Neighbour_smooth n;
        double dV_l = vel_change[counter][0];
        double dV_r = vel_change[counter][1];
        double Vl = U_l + dV_l;
        double Vr = U_r + dV_r;
        if(Vl>1) Vl=1.0;
        if(Vl<0.4) Vl=0.4;
        if(Vr>1) Vr=1.0;
        if(Vr<0.4) Vr=0.4;
        double V = (Vl+Vr)/2;
        double w = (Vl-Vr)/d_robot;
        double theta = w*time_frame + theta_i;
        double Xf = V*cos(theta)*time_frame + Xi;
        // cout<<"vl  "<<Vl<<"vr "<<Vr;
        // cout<<"v "<<V;
        // cout<<"w  "<<w;
        // cout<<"theta"<<theta;
        // cout<<"  costheta  "<<cos(theta);
        // cout<<"  xf  "<<Xf<<endl;
        // cout<<" time frame "<<time_frame;
        double Yf = V*sin(theta)*time_frame + Yi;
        //cout<<"  yf "<<Yf<<"pos "<<Yf*width + Xf<<endl;
        if(!((Xf>=0)&&(Xf<width)&&(Yf>=0)&&(Yf<height))) continue;
        n.pos = width*Yf + Xf;
        n.orientation = correct_deg(to_deg(theta));
        n.d = V*time_frame;
        n.c = abs(dV_l) + abs(dV_r);
        n.V_l = Vl;
        n.V_r = Vr;
        neighbours.push_back(n);
    }
    return neighbours;
}


};