/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

using namespace std;

class IterSearchHandle{
private:
    int iter;
    float increment;
    float x0,y0,x1,y1;
    float cur_x, cur_y;
    bool closeToEdgeX, closeToEdgeY;
    bool left_right;

public:
    IterSearchHandle(float inc, float x0, float y0, float x1, float y1){
        this->increment = inc;
        this->x0 = x0;
        this->y0 = y0;
        this->x1 = x1;
        this->y1 = y1;
        this->iter = 0;
        this->cur_x = 0;
        this->cur_y = 0;
        closeToEdgeX = true;
        closeToEdgeY = true;
        left_right = true;
    }

    ~IterSearchHandle(){
        ;
    }

    float updateX(){
        float ans;
        if(cur_x < abs(x1-x0)){
            ans = increment;
        }else{
            ans = -increment;
        }
        
        return ans;
    }

    float updateY(){
        float ans;
        if(cur_y < abs(y1-y0)){
            ans = increment;
        }else{
            ans = -increment;
        }
        return ans;
    }

    float updateX_zigzag(){
        float ans;
        if(left_right){
            ans = increment;
        }else{
            ans = -increment;
        }
        // if(closeToEdgeX){
        //     ans = ans / 3;
        // }
        return ans;
    }

    float updateY_zigzag(){
        float ans;
        if(closeToEdgeX){
            if(cur_y < abs(y1-y0)){
                ans = 2 * increment;
            }else{
                ans = 0.0;
            }
        }
        else{
            ans = 0.0;
        }
        return ans;
    }

    vector<float> update(){
        vector<float> ans;
        float limit = 2.0;
        float up_x = updateX_zigzag();
        float up_y = updateY_zigzag();
        ans.push_back(up_x);
        ans.push_back(up_y);

        iter++;
        cur_x += up_x;
        cur_y += up_y;

        if(abs(cur_x) < limit || abs(cur_x - abs(x1-x0)) < limit){
            closeToEdgeX = true;
        } else {
            closeToEdgeX = false;
        }
        if(abs(cur_y) < limit || abs(cur_y - abs(y1-y0)) < limit){
            closeToEdgeY = true;
        } else {
            closeToEdgeY = false;
        }
        if(!left_right && abs(cur_x) < 0.1){
            left_right = true;
        }
        if(left_right && abs(cur_x - abs(x1-x0)) < 0.1){
            left_right = false;
        }
        return ans;
    }
    
    bool searchComplete(){
        return false;
    }
};



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int iter = 0, flag = 0, wait_time = 200;
    float velocity = 1.0;
    float increment = velocity/20.0;
    int aim_x = 5, aim_y = 5;
    IterSearchHandle searcher(
        increment, aim_x, aim_y, aim_x + 60, aim_y + 60);

    for(int j = 100; j > 0; j--){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        iter++;

        if (!flag && iter - wait_time >= aim_x / increment
                  && iter - wait_time >= aim_y / increment){
            flag = 1;
        } else {
            ;
        }

        if(iter > wait_time){
            if(!flag){
                pose.pose.position.x += increment;
                pose.pose.position.y += increment;
            } else {
                //ROS_INFO("Arrived");
                if(flag == 1){
                    if(iter > wait_time + 100){
                        flag = 2;
                        ROS_INFO("Arrived");
                    }
                } else {
                    vector<float> increases = searcher.update();
                    pose.pose.position.x += increases[0];
                    pose.pose.position.y += increases[1];
                }
            }
        }
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

