#include "RAstar.h"

namespace globalplanner{
///// RAstar methods:

vector<int> RAstar::search_path(const int& src, const int& dest){
    auto start_clock1 = std::chrono::steady_clock::now();

    vector<int> path;
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

    //double g_score[maplen];
    //for(int loopvar=0;loopvar<maplen;loopvar++) g_score[loopvar] = DBL_MAX;
    vector<double> g_score(maplen,DBL_MAX);
    g_score[src] = 0;
    int i = src;
    double f_i = 0.0;

    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> toExplore;
    toExplore.emplace(0.0,i);

    int early_successor = -1;
    while(!toExplore.empty()){
        if (early_successor !=-1) i = early_successor;
        else{
            const Pair& p = toExplore.top();
            i = get<1>(p);
            toExplore.pop();
        }
	
	    // if(g_score[i]>30000){
	    //     path = tracePath(src,i,g_score);
	    //     //ROS_INFO("not finding full path. uncomment);
	    //     return path;
	    // }		
	
        early_successor = -1;
        double lowest_neighbour = get<0>(toExplore.top());

        vector<int> neighbours = findNeighbours(i);
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour = *loopvar;
            if(g_score[neighbour]== DBL_MAX){
		        
                g_score[neighbour] = g_score[i] + heuristic(i,neighbour);
                double h_new = heuristic(dest,neighbour);
                f_i = g_score[neighbour] + (g_score[neighbour]<h_new? 1.0000001 : 1.0001)*h_new + avg_kernel_cost(neighbour,1);

                if(lowest_neighbour > f_i){
                    if(lowest_neighbour == get<0>(toExplore.top())){
                        lowest_neighbour = f_i;
                        early_successor = neighbour;
                    }
                    else{
                        double tempp = f_i;
                        f_i = lowest_neighbour;
                        lowest_neighbour = tempp;
                        tempp = early_successor;
                        early_successor = neighbour;
                        neighbour = (int)tempp;
                    }
                }
                toExplore.emplace(f_i,neighbour);
            }

	    if(g_score[dest]!=DBL_MAX)
	    {	
		auto end_clock1 = std::chrono::steady_clock::now();
    		auto diff_time = end_clock1 - start_clock1;
    		cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for rastar" << endl;
		
		path=tracePath(src,dest, g_score);

		start_clock1 = std::chrono::steady_clock::now();
	        diff_time = start_clock1 - end_clock1;
    	   	cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for trackpath" << endl;

		return path; 
	    }
        }
    }
    
    
	if(g_score[dest]==DBL_MAX)
	{
		cout << "Failure to find a path !" << endl;
		return path;
	}

    
}

vector<int> RAstar::tracePath(const int& src,const int& dest,const vector<double>& g_score){
    vector<int> path;
    path.push_back(dest);
    int current = dest;

    while(current!=src){
        vector<int> neighbours = findNeighbours(current);
        std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> tempp;
        clearQueue(tempp);
        for(auto i=neighbours.rbegin();i!=neighbours.rend();++i)
            tempp.emplace(g_score[*i],*i);
        
        Pair p = tempp.top();
        current = get<1>(p);
        path.push_back(current);
    }

    // while(current!=src){
    //     vector<int> neighbours = findNeighbours(current);
    //     vector<double> gScores;
    //     for(uint countervar=0; countervar<neighbours.size(); countervar++)
    //         gScores.push_back(g_score[neighbours[countervar]]);
    //     int minPos = distance(gScores.begin(),min_element(gScores.begin(),gScores.end()));
    //     current = neighbours[minPos];
    //     path.push_back(current);
    // }

    return path;

}


};