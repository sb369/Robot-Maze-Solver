#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include "planner/my_msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "tf2_msgs/TFMessage.h"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <functional>

#include "nodex_global_planner.h"
#include "nodex_global_algorithms.h"

class Path{
    private:
        int width,height,map_len;
        double startx,starty,goalx=-1,goaly,resolution,originX,originY;
        int startAngle,goalAngle;
        double origin_shiftx,origin_shifty;
        vector<int> costmap;
        int tf_count=0; // dont remove. ignore
    public:
        ros::Publisher pub;
        
        ros::NodeHandle nh;

        Path(ros::NodeHandle& n): nh(n){
            //ROS_INFO("inside constructor");
            
            pub = nh.advertise<nav_msgs::Path>("/path",100);

            
        }
        void callback1(const nav_msgs::OccupancyGrid::ConstPtr& cost_map){
            ROS_INFO("callback1");
            this->resolution = cost_map->info.resolution;
            this->width = cost_map->info.width;
            this->height = cost_map->info.height;
            this->originX = cost_map->info.origin.position.x;
            this->originY = cost_map->info.origin.position.y;
            this->map_len = this->width*this->height;

            // for(auto row = cost_map.data.begin(); row!=cost_map.data.end();++row){
            //     for(auto col = row->begin(); col!= row->end; ++col){
            //         costmap.push_back((*col) * 2.53);
            //     }
            // }
            cout<<cost_map->data.size()<<"size of data"<<endl;
            //auto lol = cost_map->data;
            //cout<<typeid(lol).name();
            for(int row = 0; row<cost_map->data.size();++row){
                costmap.push_back((cost_map->data[row]) * 2.53);
            }
            cout<<costmap.size()<<"size of myyyvec"<<endl;

            //costmap = cost_map.data;
            //double myconstant = {2.53};
            //std::transform(costmap.begin(),costmap.end(),costmap.begin(),[&myconstant](auto& c){return c*myconstant;});
        }
        void callback4(const tf2_msgs::TFMessage::ConstPtr& ttt){
            ROS_INFO("callback4");
            if(tf_count<1){
                for(auto loopvar = ttt->transforms.rbegin(); loopvar!=ttt->transforms.rend(); ++loopvar){
		            if((*loopvar).header.frame_id == "map"){
                        this->origin_shiftx = ttt->transforms[0].transform.translation.x;
                        this->origin_shifty = ttt->transforms[0].transform.translation.y;
                        ROS_INFO("%f this is originx input from tf",ttt->transforms[0].transform.translation.x);
                        ROS_INFO_STREAM("this is originx input from tf: "<< ttt->transforms[0].header.frame_id);
                        ROS_INFO_STREAM("this is originx input from tf: "<< ttt->transforms[0].child_frame_id);
                        tf_count++;
                        break;
		            }
		        }
                // this->origin_shiftx = ttt->transforms[0].transform.translation.x;
                // this->origin_shifty = ttt->transforms[0].transform.translation.y;
                // ROS_INFO("%f this is originx input from tf",ttt->transforms[0].transform.translation.x);
                // ROS_INFO_STREAM("this is originx input from tf: "<< ttt->transforms[0].header.frame_id);
                // ROS_INFO_STREAM("this is originx input from tf: "<< ttt->transforms[0].child_frame_id);
                // tf_count++;
            }
        }
        void callback2(const nav_msgs::Odometry::ConstPtr& od){
            ROS_INFO("callback2");
            this->startx = od->pose.pose.position.x;
            this->starty = od->pose.pose.position.y;
            tf::Quaternion qq(od->pose.pose.orientation.x,od->pose.pose.orientation.y,od->pose.pose.orientation.z,od->pose.pose.orientation.w);
            double roll,pitch,yaw;
            tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
            this->startAngle = int(correct_deg(to_deg(yaw)));
        }
        void callback3(const geometry_msgs::PoseStamped::ConstPtr& gg){
            ROS_INFO("callback3");
            this->goalx = gg->pose.position.x;
            this->goaly = gg->pose.position.y;
            tf::Quaternion qq(gg->pose.orientation.x,gg->pose.orientation.y,gg->pose.orientation.z,gg->pose.orientation.w);
            double roll,pitch,yaw;
            tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
            this->goalAngle = int(correct_deg(to_deg(yaw)));
            // ROS_WARN("roll pitch yaw: %f %f %f",roll,pitch,yaw);
        }
        void getCoordinate(float& x,float& y){
            x = x - this->originX;
            y = y - this->originY;
        }

        int convertToCellIndex(float x, float y){ //taking world coords
            int cellInd;
            float newX = x / this->resolution; //this is map coords now
            float newY = y / this->resolution; 
            cellInd = getCellIndex(newY,newX);
            return cellInd;
        }
        int getCellIndex(int i, int j){ // pass (i,j)=(y,x)
            return (this->width*i + j);
        }
        bool isPointInside(float x, float y){
            bool valid = true;
            if(x>(this->width*this->resolution) || y>(this->height*this->resolution) || x<0 || y<0)
                valid = false;
            return valid;
        }
        // vector<geometry_msgs::PoseStamped> makePath(){
        //     //auto start_clock = std::chrono::steady_clock::now();
        //     //cout<<"HELLO WORLLLLLLDDDDD!!!";
        //     //ROS_INFO("XDDDDD make plan is executed");
        //     vector<geometry_msgs::PoseStamped> plan;
        //     plan.clear();

        //     // geometry_msgs::PoseStamped xxx;
        //     // xxx.pose.position.x = 2;
        //     // plan.push_back(xxx);
        //     // return plan;

        //     if(goalx==-1 || this->startx==this->goalx || this->starty==this->goaly) return plan;
        //     cout<<"resolution: "<<this->resolution<<"\n";
        //     // ADD 1 TO RECTIFY ODOM 
        //     float startX = this->startx + this->origin_shiftx; //world coords
        //     float startY = this->starty + this->origin_shifty;
        //     //cout<<"It might throw segmentation";
        //     float goalX = this->goalx + this->origin_shiftx; // add because we have to counter the step.
        //     float goalY = this->goaly + this->origin_shifty;
        //     cout<<"\nstartx: "<<startX<<", starty: "<<startY<<"\n";
        //     cout<<"goalx: "<<goalX<<",goalY: "<<goalY<<"\n";
        //     cout<<"origin: "<<this->originX<<",originY: "<<this->originY<<"\n";

            
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
        //         return plan;
        //     }
        //     //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y);
        //     //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",startX,startY,goalX,goalY);
        //     vector<int> bestPath;
        //     globalplanner::Astar obj(width,height,map_len,costmap);
        //     bestPath = obj.search_path(startCell,goalCell);
        //     //ROS_INFO("checkpoint 3");
        //     if(bestPath.size()>0){
        //     //ROS_INFO("checkpoint 4");
        //     //float pathlen = -1; // because prev_index is start
        //     //int prev_index = bestPath.back();
        //     for(auto loopvar = bestPath.rbegin(); loopvar!=bestPath.rend(); ++loopvar){
        //         int index = *loopvar;

        //         float x = ((index%this->width) * this->resolution) + this->originX; //this is world coord
        //         float y = ((index/this->width) * this->resolution) + this->originY;

        //         geometry_msgs::PoseStamped pose;
        //         pose.header.frame_id = "map";

        //         pose.pose.position.x = x;
        //         pose.pose.position.y = y;
        //         pose.pose.position.z = 0.0;

        //         pose.pose.orientation.x = 0.0;
        //         pose.pose.orientation.y = 0.0;
        //         pose.pose.orientation.z = 0.0;
        //         pose.pose.orientation.w = 1.0;
        //         //cout<<index%width<<","<<index/width<<"->"; //print path
        //         //pathlen += 1.4142*abs((prev_index%this->width - index%this->width)*(prev_index/this->width - index/this->width)); //diagonal
        //         //pathlen += 1 - abs((prev_index%this->width - index%this->width)*(prev_index/this->width - index/this->width)); //direct
            
        //     //if(prev_index==index) cout<<"same indd"<<endl;
        //         //prev_index = index;

        //         plan.push_back(pose);
        //     }
        //     //auto end_clock = std::chrono::steady_clock::now();
        //     //auto diff_time = end_clock - start_clock;
        //     //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for complete" << endl;
        //     //cout <<"pathlen "<<pathlen;
        //     //ROS_INFO("checkpoint 5");
        //     return plan;
        
        //     }
        //     else{
        //         ROS_WARN("planner failed to find the path. choose another goal");
        //         return plan;
        //     }

        // }
        vector<geometry_msgs::PoseStamped> makePath(){
            //auto start_clock = std::chrono::steady_clock::now();
            //cout<<"HELLO WORLLLLLLDDDDD!!!";
            //ROS_INFO("XDDDDD make plan is executed");
            vector<geometry_msgs::PoseStamped> plan;
            plan.clear();

            // geometry_msgs::PoseStamped xxx;
            // xxx.pose.position.x = 2;
            // plan.push_back(xxx);
            // return plan;

            if(goalx==-1 || this->startx==this->goalx || this->starty==this->goaly) return plan;
            cout<<"resolution: "<<this->resolution<<"\n";
            // ADD 1 TO RECTIFY ODOM 
            float startX = this->startx + this->origin_shiftx; //world coords
            float startY = this->starty + this->origin_shifty;
            //cout<<"It might throw segmentation";
            float goalX = this->goalx + this->origin_shiftx; // add because we have to counter the step.
            float goalY = this->goaly + this->origin_shifty;
            cout<<"\nstartx: "<<startX<<", starty: "<<startY<<"\n";
            cout<<"goalx: "<<goalX<<",goalY: "<<goalY<<"\n";
            cout<<"origin: "<<this->originX<<",originY: "<<this->originY<<"\n";

            
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
                return plan;
            }
            //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y);
            //ROS_INFO("checkpoint 2 (%f,%f)->(%f,%f)",startX,startY,goalX,goalY);
            vector<Index_smoothPath> bestPath;
            globalplanner::HAstar_park obj(width,height,map_len,costmap);
            Index ha_start(startCell,this->startAngle);
            Index ha_goal(goalCell,this->goalAngle);
            bestPath = obj.search_path(ha_start,ha_goal);
            //ROS_INFO("checkpoint 3");
            if(bestPath.size()>0){
            //ROS_INFO("checkpoint 4");
            //float pathlen = -1; // because prev_index is start
            //int prev_index = bestPath.back();
            int flag = 0;
            // cout<<"start path: "<<bestPath.back().pos%width<<" , "<<bestPath.back().pos/width<<endl;
            for(auto loopvar = bestPath.rbegin(); loopvar!=bestPath.rend(); ++loopvar){
                int index = loopvar->pos;
                int path_orient = loopvar->orientation;
                double path_dd = loopvar->dd;
                double path_steer_angle = loopvar->steer_angle;
                ActualPos actual_pose = loopvar->pose;
                tf::Quaternion qq;
                // cout<<"orient->"<<path_orient<<",steer: "<<path_steer_angle;

                double path_divisions = 10;
                double L = 10;

                // if(flag==1 && index!=bestPath[0].pos){
                //     double current_x = ((loopvar-1)->pos)%this->width;
                //     double current_y = ((loopvar-1)->pos)/this->width;
                //     double theta = to_rads((loopvar-1)->orientation);
                    
                //     path_steer_angle = atan(2*sin(to_rads((loopvar+1)->orientation - (loopvar-1)->orientation))*(L/path_dd)*0.5);

                //     for(int counter_i = 1; counter_i<=path_divisions; counter_i++){
                //         double alpha = path_steer_angle;
                //         double beta = ((path_dd * counter_i/path_divisions) / L) * tan(alpha);

                //         //calculate new pos
                //         double next_x, next_y;
                //         if(abs(beta)<0.00001f){
                //             next_x = current_x + (path_dd * counter_i/path_divisions) * cos(theta);
                //             next_y = current_y + (path_dd * counter_i/path_divisions) * sin(theta);
                //         }

                //         else{
                //             double R = (path_dd * counter_i/path_divisions) / beta;

                //             double cx = current_x - sin(theta) * R;
                //             double cy = current_y + cos(theta) * R;

                //             next_x = cx + sin(theta + beta) * R;
                //             next_y = cy - cos(theta + beta) * R;
                //         }
                //         qq.setRPY(0,-0,deg_to_rads_inrange(int(correct_deg(to_deg(theta + beta)))));

                //         float x = (next_x * this->resolution) + this->originX; //this is world coord
                //         float y = (next_y * this->resolution) + this->originY;

                //         geometry_msgs::PoseStamped pose;
                //         pose.header.frame_id = "map";

                //         pose.pose.position.x = x;
                //         pose.pose.position.y = y;
                //         pose.pose.position.z = 0.0;

                //         pose.pose.orientation.x = qq[0];
                //         pose.pose.orientation.y = qq[1];
                //         pose.pose.orientation.z = qq[2];
                //         pose.pose.orientation.w = qq[3];

                //         plan.push_back(pose);

                //     }
                // }



                // if(flag==1 && index!=bestPath[0].pos){
                //     path_dd = sqrt((loopvar->pos%width - (loopvar-1)->pos%width)*(loopvar->pos%width - (loopvar-1)->pos%width) + (loopvar->pos/width - (loopvar-1)->pos/width)*(loopvar->pos/width - (loopvar-1)->pos/width));
                    
                //     // //try:
                //     // double betaaa = (path_dd/L) * tan(path_steer_angle);
                //     // double beta = (loopvar)->orientation - (loopvar-1)->orientation;
                //     // if(correct_deg(to_deg(betaaa)) - correct_deg(to_deg(beta)) >= 180) beta = 2*M_PI*int(betaaa/(2*M_PI)) + 2*M_PI + to_rads(correct_deg(to_deg(beta)));
                //     // else if(correct_deg(to_deg(beta)) - correct_deg(to_deg(betaaa)) >= 180) beta = 2*M_PI*int(betaaa/(2*M_PI)) - 2*M_PI + to_rads(correct_deg(to_deg(beta)));
                //     // else beta = 2*M_PI*int(betaaa/(2*M_PI)) + to_rads(correct_deg(to_deg(beta)));
                //     // path_steer_angle = atan(beta * (L/path_dd));
                //     // cout<<endl<<"approx steer: "<<path_steer_angle<<endl;

                //     for(int counter_i = 1; counter_i<=path_divisions; counter_i++){
                //         geometry_msgs::PoseStamped prev_pose = plan.back();
                //         double current_x = (prev_pose.pose.position.x - this->originX)/this->resolution;
                //         double current_y = (prev_pose.pose.position.y - this->originY)/this->resolution;
                //         tf::Quaternion qq_temp(prev_pose.pose.orientation.x,prev_pose.pose.orientation.y,prev_pose.pose.orientation.z,prev_pose.pose.orientation.w);
                //         double roll,pitch,yaw;
                //         tf::Matrix3x3(qq_temp).getRPY(roll, pitch, yaw);
                //         int prev_angle = int(correct_deg(to_deg(yaw)));
                        
                //         double theta = to_rads(prev_angle);
                //         path_steer_angle = atan(1*sin(to_rads((loopvar+1)->orientation - prev_angle))*(L/path_dd)*0.5);
                //         // cout<<"new steer angle: "<<path_steer_angle;

                //         // cout<<"  x: "<<current_x<<"  y: "<<current_y<<"  theta: "<<prev_angle<<" --";
                    
                //         double alpha = path_steer_angle;
                //         double beta = ((path_dd /path_divisions) / L) * tan(alpha);

                        

                //         //calculate new pos
                //         double next_x, next_y;
                        
                //         if(abs(beta)<0.00001f){
                //             next_x = current_x + (path_dd/path_divisions) * cos(theta);
                //             next_y = current_y + (path_dd/path_divisions) * sin(theta);
                //         }

                //         else{
                //             double R = (path_dd /path_divisions) / beta;

                //             double cx = current_x - sin(theta) * R;
                //             double cy = current_y + cos(theta) * R;

                //             next_x = cx + sin(theta + beta) * R;
                //             next_y = cy - cos(theta + beta) * R;
                //         }
                //         qq.setRPY(0,-0,deg_to_rads_inrange(int(correct_deg(to_deg(theta + beta)))));

                //         float x = (round_to_decimal(next_x) * this->resolution) + this->originX; //this is world coord
                //         float y = (round_to_decimal(next_y) * this->resolution) + this->originY;

                //         geometry_msgs::PoseStamped pose;
                //         pose.header.frame_id = "map";

                //         pose.pose.position.x = x;
                //         pose.pose.position.y = y;
                //         pose.pose.position.z = 0.0;

                //         pose.pose.orientation.x = qq[0];
                //         pose.pose.orientation.y = qq[1];
                //         pose.pose.orientation.z = qq[2];
                //         pose.pose.orientation.w = qq[3];

                //         cout<<"  pushing  x: "<<x<<"  y: "<<y<<"  theta: "<<correct_deg(to_deg(theta + beta))<<" --"<<endl;
                //         plan.push_back(pose);

                //     }
                // }
                // cout<<endl;
                qq.setRPY(0,-0,deg_to_rads_inrange(to_deg(actual_pose.orientation)));

                float x = (actual_pose.x * this->resolution) + this->originX; //this is world coord
                float y = (actual_pose.y * this->resolution) + this->originY;

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = qq[0];
                pose.pose.orientation.y = qq[1];
                pose.pose.orientation.z = qq[2];
                pose.pose.orientation.w = qq[3];
                //cout<<index%width<<","<<index/width<<"->"; //print path
                //pathlen += 1.4142*abs((prev_index%this->width - index%this->width)*(prev_index/this->width - index/this->width)); //diagonal
                //pathlen += 1 - abs((prev_index%this->width - index%this->width)*(prev_index/this->width - index/this->width)); //direct
            
            //if(prev_index==index) cout<<"same indd"<<endl;
                //prev_index = index;
                // if(flag==0 || index==bestPath[0].pos){
                    flag = 1;
                    // cout<<"pushing  x: "<<x<<"  y: "<<y<<"  theta: "<<path_orient<<" --"<<endl;
                    plan.push_back(pose);
                // }   
            }
            //auto end_clock = std::chrono::steady_clock::now();
            //auto diff_time = end_clock - start_clock;
            //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for complete" << endl;
            //cout <<"pathlen "<<pathlen;
            //ROS_INFO("checkpoint 5");
            return plan;
        
            }
            else{
                ROS_WARN("planner failed to find the path. choose another goal");
                return plan;
            }

        }

};
//std::vector<geometry_msgs::PoseStamped> temp_msg;
//temp_msg = std::vector<geometry_msgs::PoseStamped>();
//old_msg.poses.clear();


// void callback(const planner::my_msg::ConstPtr& new_msg){
//     ROS_INFO("Heyyy subscriber got a new msg ");
//     //temp_msg = new_msg->poses;
//     pub = n.advertise<planner::my_msg>("/globalPlanner/Path",10);
//     if(new_msg->poses.size()>0) pub.publish(*new_msg);
// }


int main(int argc, char **argv){
    ros::init(argc,argv,"nodex");
    
  
    ros::NodeHandle nh1;
    ros::Subscriber sub1,sub2,sub3,sub4;
    Path obj1(nh1);
    sub1 = nh1.subscribe("/move_base/global_costmap/costmap",100,&Path::callback1,&obj1);
    sub4 = nh1.subscribe("/tf_static",100,&Path::callback4,&obj1);
    // boost::shared_ptr<tf2_msgs::TFMessage> ttt = (ros::topic::waitForMessage<tf2_msgs::TFMessage>("/tf2_static",nh1));
    // obj1.callback4(ttt);
    while(ros::ok()){
        //pub_msg.poses.clear();
        
        sub3 = nh1.subscribe("/move_base_simple/goal",10,&Path::callback3,&obj1);
        
        sub2 = nh1.subscribe("/odom",10,&Path::callback2,&obj1);
        

        nav_msgs::Path pub_msg;
        pub_msg.poses.clear();
        pub_msg.header.frame_id = "map";
        pub_msg.poses = (obj1.makePath());
        if(pub_msg.poses.size()>0){
            obj1.pub.publish(pub_msg);
            ROS_INFO("pubbb");
        }
        ros::spinOnce();
        
    }


    // sub3 = nh1.subscribe("/move_base_simple/goal",1000,&Path::callback3,&obj1);
    // sub1 = nh1.subscribe("/move_base/global_costmap/costmap",100,&Path::callback1,&obj1);
    // sub2 = nh1.subscribe("/odom",1,&Path::callback2,&obj1);
    

    // nav_msgs::Path pub_msg;
    // pub_msg.poses.clear();
    // pub_msg.poses = (obj1.makePath());
    // if(pub_msg.poses.size()>0){
    //     obj1.pub.publish(pub_msg);
    //     ROS_INFO("pubbb");
    // }
    // ros::spin();
    

    return 0;

}