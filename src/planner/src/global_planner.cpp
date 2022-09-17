#include "../include/global_planner.h"
// #include "global_algorithms.h"
#include "../include/PlannerAlgorithm.h"
#include "../include/HAstar_park.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(globalplanner::GlobalPlanner, nav_core::BaseGlobalPlanner)



int map_len;
//auto heuristic_time = 0;

namespace globalplanner{

///// GlobalPlanner methods:

GlobalPlanner::GlobalPlanner(){

}

GlobalPlanner::GlobalPlanner(ros::NodeHandle& n){
    my_nh = n;
    ROS_INFO("inside nodehandle constructor");
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name,costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);
	
	
	    my_nh = private_nh;
	    pubb = my_nh.advertise<planner::my_msg>("shreypath_topic",10);

        accn = 1.0;
        d_robot = 0.16;
        time_frame = 1 ; 

        originX = costmap_->getOriginX();
        originY = costmap_->getOriginY(); //these are world coords

        width = costmap_->getSizeInCellsX(); //these are map ones
	    height = costmap_->getSizeInCellsY();
	    resolution = costmap_->getResolution();
	    map_len = width*height;

        ROS_INFO("Global Planner by Shrey initialized successfully");
        initialized_ = true;
    }
    else
        ROS_WARN("Global Planner already initialized");
}


/// normal makePlan function (ha_star):

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    tf::Quaternion qq(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w);
    double roll,pitch,yaw;
    // ROS_WARN("quaternion: %f %f %f %f",qq[0],qq[1],qq[2],qq[3]);
    // tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
    // double yaww = (yaw*180.0)/M_PI;
    // ROS_WARN("roll pitch yaw: %f %f %f",roll,pitch,yaww);  
    // qq.setRPY(0,-0,yaw);
    // ROS_WARN("quaternion: %f %f %f %f",qq[0],qq[1],qq[2],qq[3]);  

    tf::Quaternion qqs(start.pose.orientation.x,start.pose.orientation.y,start.pose.orientation.z,start.pose.orientation.w);
    roll,pitch,yaw;
    tf::Matrix3x3(qqs).getRPY(roll, pitch, yaw);
    int startAngle = int(correct_deg(to_deg(yaw)));
    tf::Quaternion qqg(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w);
    tf::Matrix3x3(qqg).getRPY(roll, pitch, yaw);
    int goalAngle = int(correct_deg(to_deg(yaw)));

    auto start_clock = std::chrono::steady_clock::now();
    //cout<<"HELLO WORLLLLLLDDDDD!!!";
    ROS_INFO("XDDDDD make plan is executed");
    plan.clear();
    
    float startX = start.pose.position.x; //world coords
    float startY = start.pose.position.y;
    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;
    // cout<<"\nstartx: "<<startX<<", starty: "<<startY<<endl;
    // cout<<"goalx: "<<goalX<<",goalY: "<<goalY<<endl;
    // cout<<"origin: "<<originX<<",originY: "<<originY<<endl;
    // cout<<"\nresolution "<<resolution<<endl;

    getCoordinate(startX,startY); //still world coords but wrt origin
    getCoordinate(goalX,goalY);
    //ROS_INFO("checkpoint 1");
    int startCell, goalCell; //these will be cell ones

    if(isPointInside(startX,startY) && isPointInside(goalX,goalY)){
        startCell = convertToCellIndex(startX,startY); // y is rows x is cols
        goalCell = convertToCellIndex(goalX,goalY);
    }
    else{
        ROS_WARN("start or goal is out of the map");
        return false;
    }
    
    
    /// CHECKPOINT METHOD
    // iter++;
    // if(iter%100==0 || prev_goal!=goalCell){
    //     iter = 1;
    //     checkpoint = -1;
    //     full_dist = 0;
    //     temp_dist = 0;
    // }
	// prev_goal = goalCell;
	// cout<<"full_dist: "<<full_dist<<endl;
    // bool checkpt_method = true;
    // if(full_dist <= 1.5*checkpt_dist){
    //     checkpt_method = false;
    // }
    // if(checkpt_method){
    //     if(temp_dist>10){
    //         goalCell = checkpoint;
	// 	ROS_INFO("goalcell now checkpoint");
    //     }
    //     else{
    //         iter = 1;
    //         checkpoint = -1;
    //         full_dist = 0;
    //         temp_dist = 0;
    //     }
        
    // }
    
    // cout<<"\n iter:"<<iter<<endl;
    //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y);
    // ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",startX,startY,goalX,goalY);
    vector<Index_smoothPath> bestPath;
    HAstar_park obj(width,height,map_len,costmap_);
    // ROS_INFO("checkpoint 3");
    Index ha_start(startCell,startAngle);
    Index ha_goal(goalCell,goalAngle);
    bestPath = obj.search_path(ha_start,ha_goal);

    //bestPath.clear();
    //for(int loopvar = 0; loopvar<10; loopvar++) bestPath.push_back(startCell+loopvar);
    //bestPath.push_back(goalCell);
    //bestPath.push_back(startCell);

    // ROS_INFO("checkpoint 3 (%d,%d)->(%d,%d)",startCell%width,startCell/width,goalCell%width,goalCell/width);
    /*if(startCell==goalCell){
	int index = goalCell+1;
	
        float x = ((index%width) * resolution) + originX; //this is world coord
        float y = ((index/width) * resolution) + originY;
	ROS_INFO("(%f,%f)",x,y);
        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }*/
    if(bestPath.size()>0){
	    //ROS_INFO("checkpoint 4");
	
	    float pathlen = -1; // because prev_index is start
        int prev_index = bestPath.back().pos;
        int temp_flag = 0;
        for(auto loopvar = bestPath.rbegin(); loopvar!=bestPath.rend(); ++loopvar){
            int index = loopvar->pos;
            int path_orient = loopvar->orientation;
            double path_dd = loopvar->dd;
            double path_steer_angle = loopvar->steer_angle;
            ActualPos actual_pose = loopvar->pose;
            qq.setRPY(0,-0,deg_to_rads_inrange(actual_pose.orientation));

            float x = (actual_pose.x * resolution) + originX; //this is world coord
            float y = (actual_pose.y * resolution) + originY;

            geometry_msgs::PoseStamped pose = goal;

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = qq[0];
            pose.pose.orientation.y = qq[1];
            pose.pose.orientation.z = qq[2];
            pose.pose.orientation.w = qq[3];
            // pose.pose.orientation.x = 0.0;
            // pose.pose.orientation.y = 0.0;
            // pose.pose.orientation.z = 0.0;
            // pose.pose.orientation.w = 1.0;
	        //cout<<index%width<<","<<index/width<<"->";
	        pathlen += 1.4142*abs((prev_index%width - index%width)*(prev_index/width - index/width)); //diagonal
            pathlen += 1 - abs((prev_index%width - index%width)*(prev_index/width - index/width)); //direct
	    
        //     if(pathlen>checkpt_dist && temp_flag==0){
        //         checkpoint = index;
        //         temp_dist = pathlen;
		// temp_flag=1;
        //     }
	    //if(prev_index==index) cout<<"same indd"<<endl;
            prev_index = index;

            plan.push_back(pose);
        }

        // if(iter==1) full_dist = pathlen;
        // else temp_dist = pathlen;
            
        // planner::my_msg msg;
        // msg.poses = plan;
        // pubb.publish(msg);
        auto end_clock = std::chrono::steady_clock::now();
        auto diff_time = end_clock - start_clock;
        cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for complete" << endl;
	    cout <<"pathlen "<<pathlen;
	    //ROS_INFO("checkpoint 5");
        return true;
	
    }
    else{
        ROS_WARN("planner failed to find the path. choose another goal");
        return false;
    }
}


// /// normal makePlan function (a star) :

// bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
//     if (!initialized_){
//         ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
//         return false;
//     }

//     tf::Quaternion qq(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w);
//     double roll,pitch,yaw;
//     // ROS_WARN("quaternion: %f %f %f %f",qq[0],qq[1],qq[2],qq[3]);
//     // tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
//     // double yaww = (yaw*180.0)/M_PI;
//     // ROS_WARN("roll pitch yaw: %f %f %f",roll,pitch,yaww);  
//     // qq.setRPY(0,-0,yaw);
//     // ROS_WARN("quaternion: %f %f %f %f",qq[0],qq[1],qq[2],qq[3]);  

//     auto start_clock = std::chrono::steady_clock::now();
//     //cout<<"HELLO WORLLLLLLDDDDD!!!";
//     ROS_INFO("XDDDDD make plan is executed");
//     plan.clear();
    
//     float startX = start.pose.position.x; //world coords
//     float startY = start.pose.position.y;
//     float goalX = goal.pose.position.x;
//     float goalY = goal.pose.position.y;
//     cout<<"\nstartx: "<<startX<<", starty: "<<startY<<endl;
//     cout<<"goalx: "<<goalX<<",goalY: "<<goalY<<endl;
//     cout<<"origin: "<<originX<<",originY: "<<originY<<endl;
//     cout<<"\nresolution "<<resolution<<endl;

//     getCoordinate(startX,startY); //still world coords but wrt origin
//     getCoordinate(goalX,goalY);
//     //ROS_INFO("checkpoint 1");
//     int startCell, goalCell; //these will be cell ones

//     if(isPointInside(startX,startY) && isPointInside(goalX,goalY)){
//         startCell = convertToCellIndex(startX,startY); // y is rows x is cols
//         goalCell = convertToCellIndex(goalX,goalY);
//     }
//     else{
//         ROS_WARN("start or goal is out of the map");
//         return false;
//     }
    
    
//     /// CHECKPOINT METHOD
//     // iter++;
//     // if(iter%100==0 || prev_goal!=goalCell){
//     //     iter = 1;
//     //     checkpoint = -1;
//     //     full_dist = 0;
//     //     temp_dist = 0;
//     // }
// 	// prev_goal = goalCell;
// 	// cout<<"full_dist: "<<full_dist<<endl;
//     // bool checkpt_method = true;
//     // if(full_dist <= 1.5*checkpt_dist){
//     //     checkpt_method = false;
//     // }
//     // if(checkpt_method){
//     //     if(temp_dist>10){
//     //         goalCell = checkpoint;
// 	// 	ROS_INFO("goalcell now checkpoint");
//     //     }
//     //     else{
//     //         iter = 1;
//     //         checkpoint = -1;
//     //         full_dist = 0;
//     //         temp_dist = 0;
//     //     }
        
//     // }
    
//     cout<<"\n iter:"<<iter<<endl;
//     //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y);
//     //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",startX,startY,goalX,goalY);
//     vector<int> bestPath;
//     Astar obj(width,height,map_len,costmap_);
//     bestPath = obj.search_path(startCell,goalCell);

//     //bestPath.clear();
//     //for(int loopvar = 0; loopvar<10; loopvar++) bestPath.push_back(startCell+loopvar);
//     //bestPath.push_back(goalCell);
//     //bestPath.push_back(startCell);

//     //ROS_INFO("checkpoint 3 (%d,%d)->(%d,%d)",startCell%width,startCell/width,goalCell%width,goalCell/width);
//     /*if(startCell==goalCell){
// 	int index = goalCell+1;
	
//         float x = ((index%width) * resolution) + originX; //this is world coord
//         float y = ((index/width) * resolution) + originY;
// 	ROS_INFO("(%f,%f)",x,y);
//         geometry_msgs::PoseStamped pose = goal;

//         pose.pose.position.x = x;
//         pose.pose.position.y = y;
//         pose.pose.position.z = 0.0;

//         pose.pose.orientation.x = 0.0;
//         pose.pose.orientation.y = 0.0;
//         pose.pose.orientation.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         plan.push_back(pose);
//     }*/
//     if(bestPath.size()>0){
// 	    //ROS_INFO("checkpoint 4");
	
// 	    float pathlen = -1; // because prev_index is start
//         int prev_index = bestPath.back();
//         int temp_flag = 0;
//         for(auto loopvar = bestPath.rbegin(); loopvar!=bestPath.rend(); ++loopvar){
//             int index = *loopvar;

//             float x = ((index%width) * resolution) + originX; //this is world coord
//             float y = ((index/width) * resolution) + originY;

//             geometry_msgs::PoseStamped pose = goal;

//             pose.pose.position.x = x;
//             pose.pose.position.y = y;
//             pose.pose.position.z = 0.0;

//             pose.pose.orientation.x = 0.0;
//             pose.pose.orientation.y = 0.0;
//             pose.pose.orientation.z = 0.0;
//             pose.pose.orientation.w = 1.0;
// 	        //cout<<index%width<<","<<index/width<<"->";
// 	        pathlen += 1.4142*abs((prev_index%width - index%width)*(prev_index/width - index/width)); //diagonal
//             pathlen += 1 - abs((prev_index%width - index%width)*(prev_index/width - index/width)); //direct
	    
//             if(pathlen>checkpt_dist && temp_flag==0){
//                 checkpoint = index;
//                 temp_dist = pathlen;
// 		temp_flag=1;
//             }
// 	    //if(prev_index==index) cout<<"same indd"<<endl;
//             prev_index = index;

//             plan.push_back(pose);
//         }

//         if(iter==1) full_dist = pathlen;
//         else temp_dist = pathlen;
            
//         // planner::my_msg msg;
//         // msg.poses = plan;
//         // pubb.publish(msg);
//         auto end_clock = std::chrono::steady_clock::now();
//         auto diff_time = end_clock - start_clock;
//         cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for complete" << endl;
// 	    cout <<"pathlen "<<pathlen;
// 	    //ROS_INFO("checkpoint 5");
//         return true;
	
//     }
//     else{
//         ROS_WARN("planner failed to find the path. choose another goal");
//         return false;
//     }
// }


/////// makePlan function for algos considering kinematics:

// bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
//     if (!initialized_){
//         ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
//         return false;
//     }

//     tf::Quaternion qq(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w);
//     double roll,pitch,yaw;
//     tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
//     yaw = correct_deg(to_deg(yaw));
//     ROS_WARN("roll pitch yaw: %f %f %f",roll,pitch,yaw);    



//     auto start_clock = std::chrono::steady_clock::now();
//     //cout<<"HELLO WORLLLLLLDDDDD!!!";
//     ROS_INFO("XDDDDD make plan is executed");
//     plan.clear();
    
//     float startX = start.pose.position.x; //world coords
//     float startY = start.pose.position.y;
//     float goalX = goal.pose.position.x;
//     float goalY = goal.pose.position.y;
//     cout<<"\nstartx: "<<startX<<", starty: "<<startY<<endl;
//     cout<<"goalx: "<<goalX<<",goalY: "<<goalY<<endl;
//     cout<<"origin: "<<originX<<",originY: "<<originY<<endl;
//     cout<<"\nresolution "<<resolution<<endl;

//     getCoordinate(startX,startY); //still world coords but wrt origin
//     getCoordinate(goalX,goalY);
//     //ROS_INFO("checkpoint 1");
//     int startCell, goalCell; //these will be cell ones

//     if(isPointInside(startX,startY) && isPointInside(goalX,goalY)){
//         startCell = convertToCellIndex(startX,startY); // y is rows x is cols
//         goalCell = convertToCellIndex(goalX,goalY);
//     }
//     else{
//         ROS_WARN("start or goal is out of the map");
//         return false;
//     }
    
    
//     /// CHECKPOINT METHOD
//     iter++;
//     if(iter%100==0 || prev_goal!=goalCell){
//         iter = 1;
//         checkpoint = -1;
//         full_dist = 0;
//         temp_dist = 0;
//     }
// 	prev_goal = goalCell;
// 	cout<<"full_dist: "<<full_dist<<endl;
//     bool checkpt_method = true;
//     if(full_dist <= 1.5*checkpt_dist){
//         checkpt_method = false;
//     }
//     if(checkpt_method){
//         if(temp_dist>10){
//             goalCell = checkpoint;
// 		ROS_INFO("goalcell now checkpoint");
//         }
//         else{
//             iter = 1;
//             checkpoint = -1;
//             full_dist = 0;
//             temp_dist = 0;
//         }
        
//     }
    
//     cout<<"\n iter:"<<iter<<endl;
//     //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y);
//     //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",startX,startY,goalX,goalY);
//     vector<double> initial_vel(2);
//     //if(!(smooth_prev_start.pos==startCell && smooth_prev_path.orientation==yaw)){
//     if(!(smooth_prev_start.pos==startCell)){
//         initial_vel = find_vel(1/10,d_robot,startCell%width,startCell/width,yaw,smooth_prev_start.pos%width,smooth_prev_start.pos/width,smooth_prev_start.orientation);
    
//     }
//     smooth_prev_start.pos = startCell;
//     smooth_prev_start.orientation = yaw;

//     vector<Index> bestPath;
//     Astar_smooth obj(width,height,map_len,costmap_);
//     bestPath = obj.search_path(smooth_prev_start,goalCell,accn,initial_vel,time_frame,d_robot);

//     //bestPath.clear();
//     //for(int loopvar = 0; loopvar<10; loopvar++) bestPath.push_back(startCell+loopvar);
//     //bestPath.push_back(goalCell);
//     //bestPath.push_back(startCell);

//     //ROS_INFO("checkpoint 3 (%d,%d)->(%d,%d)",startCell%width,startCell/width,goalCell%width,goalCell/width);
//     /*if(startCell==goalCell){
// 	int index = goalCell+1;
	
//         float x = ((index%width) * resolution) + originX; //this is world coord
//         float y = ((index/width) * resolution) + originY;
// 	ROS_INFO("(%f,%f)",x,y);
//         geometry_msgs::PoseStamped pose = goal;

//         pose.pose.position.x = x;
//         pose.pose.position.y = y;
//         pose.pose.position.z = 0.0;

//         pose.pose.orientation.x = 0.0;
//         pose.pose.orientation.y = 0.0;
//         pose.pose.orientation.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         plan.push_back(pose);
//     }*/
//     if(bestPath.size()>0){
// 	    //ROS_INFO("checkpoint 4");
	
// 	    float pathlen = -1; // because prev_index is start
//         int prev_index = bestPath.back().pos;
//         int temp_flag = 0;
//         for(auto loopvar = bestPath.rbegin(); loopvar!=bestPath.rend(); ++loopvar){
//             int index = loopvar->pos;
//             int theta = loopvar->orientation;

//             tf::Quaternion myQ;
            
//             myQ.setRPY(0,-0,deg_to_rads_inrange(theta));
//             // ROS_WARN("quaternion: %f %f %f %f",qq[0],qq[1],qq[2],qq[3]);

//             float x = ((index%width) * resolution) + originX; //this is world coord
//             float y = ((index/width) * resolution) + originY;

//             geometry_msgs::PoseStamped pose = goal;

//             pose.pose.position.x = x;
//             pose.pose.position.y = y;
//             pose.pose.position.z = 0.0;

//             pose.pose.orientation.x = myQ[0];
//             pose.pose.orientation.y = myQ[1];
//             pose.pose.orientation.z = myQ[2];
//             pose.pose.orientation.w = myQ[3];
// 	        //cout<<index%width<<","<<index/width<<"->";
// 	        pathlen += 1.4142*abs((prev_index%width - index%width)*(prev_index/width - index/width)); //diagonal
//             pathlen += 1 - abs((prev_index%width - index%width)*(prev_index/width - index/width)); //direct
	    
//             if(pathlen>checkpt_dist && temp_flag==0){
//                 checkpoint = index;
//                 temp_dist = pathlen;
// 		        temp_flag=1;
//             }
// 	    //if(prev_index==index) cout<<"same indd"<<endl;
//             prev_index = index;

//             plan.push_back(pose);
//         }

//         if(iter==1) full_dist = pathlen;
//         else temp_dist = pathlen;
            
//         // planner::my_msg msg;
//         // msg.poses = plan;
//         // pubb.publish(msg);
//         auto end_clock = std::chrono::steady_clock::now();
//         auto diff_time = end_clock - start_clock;
//         cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for complete" << endl;
// 	    cout <<"pathlen "<<pathlen;
// 	    //ROS_INFO("checkpoint 5");
//         return true;
	
//     }
//     else{
//         ROS_WARN("planner failed to find the path. choose another goal");
//         return false;
//     }
// }


///// additional functions:

void GlobalPlanner::getCoordinate(float& x,float& y){
    x = x - originX;
    y = y - originY;
}

int GlobalPlanner::convertToCellIndex(float x, float y){ //taking world coords
    int cellInd;
    float newX = x / resolution; //this is map coords now
    float newY = y / resolution; 
    cellInd = getCellIndex(newY,newX);
    return cellInd;
}

bool GlobalPlanner::isPointInside(float x, float y){
    bool valid = true;
    if(x>(width*resolution) || y>(height*resolution) || x<0 || y<0)
        valid = false;
    return valid;
}




};
