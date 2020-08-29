#ifndef CONTROL_H_
#define CONTROL_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include <tf/tf.h>
#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <string>

namespace rvt = rviz_visual_tools;

class IIwaControl{
public:
    IIwaControl(){
        nh_.param<std::string>("/PLANNING_GROUP",PLANNING_GROUP,"manipulator");
        nh_.param<std::string>("/frame_id",frame_id,"iiwa_link_0");
        nh_.param<double>("/move_step",move_step,0.1);
        nh_.param<double>("/lift_step",lift_step,0.1);
        nh_.param<double>("/planning_time",planning_time, 0.5);
        nh_.param<double>("/roll_step",roll_step, 0.1);
        nh_.param<double>("/pitch_step",pitch_step, 0.1);
        nh_.param<double>("/yaw_step",yaw_step, 0.1);

        ROS_INFO("LOAD THE PARAMETER");
        ROS_INFO("PLANNING_GROUP: %s",PLANNING_GROUP.c_str());
        ROS_INFO("frame_id: %s",frame_id.c_str());
        ROS_INFO("move_step: %f",move_step);
        ROS_INFO("lift_step: %f",lift_step);
        ROS_INFO("planning_time: %f", planning_time);
        ROS_INFO("roll_step: %f",roll_step);
        ROS_INFO("pitch_step: %f",pitch_step);
        ROS_INFO("yaw_step: %f",yaw_step);
        
        joy_sub_ = nh_.subscribe("/joy", 1, &IIwaControl::joyCallback, this);   

        move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        visual_tools = new moveit_visual_tools::MoveItVisualTools("panda_link0");
        joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        move_group->setPlanningTime(planning_time);

        execute_in = true;

        // DEPRECATED 
        // jump_threshold = 0.0;
        // eef_step = 0.01;       
    }

    void testToStaticPosition(){
        // To the initial pose
        geometry_msgs::PoseStamped init_pose_;
        setPose(init_pose_, frame_id, 0,0,1.266,0,0,0,1);
        move_group->setPoseTarget(init_pose_);
        evaluate_plan(*move_group);

        // To a random pose
        setPose(init_pose_, frame_id, -0.235056,-0.558337,0.932261,0.000313,0.000378,0.000614,1);
        move_group->setPoseTarget(init_pose_);
        evaluate_plan(*move_group);

        // To the pose that prepare to grasp
        setPose(init_pose_, frame_id, 0.642263,0.00152939,0.759403,0.703406,-0.0033564,0.710599,0.0160412);
        move_group->setPoseTarget(init_pose_);
        evaluate_plan(*move_group);
    }

    void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group){
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool result_ = false;

        ROS_INFO("Planning.");
        result_ = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("After planning.");
        ros::WallDuration(0.1).sleep();

        ROS_INFO("Before execute.");
        if(result_){
            group.execute(my_plan);
            ROS_INFO("Execute successful.");
        }else{
            ROS_INFO("Planning failed.");
        }
        ROS_INFO("Execute ended.");
    }

    void setPose(geometry_msgs::PoseStamped& pose_, std::string frame_id, double x, double y, double z, 
                                    double q_x, double q_y, double q_z, double q_w)
    {
        pose_.pose.position.x = x;
        pose_.pose.position.y = y;
        pose_.pose.position.z = z;
        pose_.pose.orientation.x = q_x;
        pose_.pose.orientation.y = q_y;
        pose_.pose.orientation.z = q_z;
        pose_.pose.orientation.w = q_w;
        pose_.header.frame_id = frame_id;
        pose_.header.stamp = ros::Time::now();
    }

    void testKeyBoard(){
        move_group->stop();
        char c;
        double step = 0.02;
        while(ros::ok){
            ROS_INFO("Please input a instuction(q to quit): ");
            std::cin >> c;
            
            double y_move=0, x_move=0, z_move=0;
            if(c == 'a')
                x_move -= step;
            else if(c == 'd')
                x_move += step;
            else if(c == 's')
                y_move -= step;
            else if(c == 'w')
                y_move += step;
            else if(c == 'z')
                z_move -= step;
            else if(c == 'x')
                z_move += step;
            else if(c == '1'){
                setPose(target_pose, frame_id, 0,0,1.266,0,0,0,1);
                move_group->setPoseTarget(target_pose);
                evaluate_plan(*move_group);
                continue;
            }else if(c == '2'){
                setPose(target_pose, frame_id, 0.642263,0.00152939,0.759403,0.703406,-0.0033564,0.710599,0.0160412);
                move_group->setPoseTarget(target_pose);
                evaluate_plan(*move_group);
            }
            else if(c == 'q')
                return;
            else
                continue;
            
            do{
                target_pose = move_group->getCurrentPose("iiwa_link_ee");
            }while(target_pose.pose.position.z == 0);
            
            ROS_INFO("Curren position: %f %f %f.", target_pose.pose.position.x, target_pose.pose.position.y,
                                                target_pose.pose.position.z);
            
            target_pose.pose.position.x += x_move;
            target_pose.pose.position.y += y_move; 
            target_pose.pose.position.z += z_move;


            setPose(target_pose, frame_id, target_pose.pose.position.x, target_pose.pose.position.y,target_pose.pose.position.z,
                    target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);

            move_group->setPoseTarget(target_pose);
            evaluate_plan(*move_group);
            
            ROS_INFO("Planning to: %f %f %f.", target_pose.pose.position.x, target_pose.pose.position.y,
                                            target_pose.pose.position.z);
            ROS_INFO("circle..");
            }
    }   
    

    void joy_execute(double x_move, double y_move, double z_move, double yaw_rotate, double pitch_rotate, double roll_rotate, int pose_flag){
        execute_in = false;
        ROS_INFO("===============================================================");
        move_group->stop();
        if(pose_flag == 1){
            setPose(target_pose, frame_id, 0.318115,0.002605,0.876167,-0.464508,0.443012,-0.533560,0.550715);
            move_group->setPoseTarget(target_pose);
            evaluate_plan(*move_group);
            pose_flag = 0;
            execute_in = true;
            return;
        }else if(pose_flag == 10){
            setPose(target_pose, frame_id, 0,0,1.266,0,0,0,1);
            move_group->setPoseTarget(target_pose);
            evaluate_plan(*move_group);
            pose_flag = 0;
            execute_in = true;
            return;
        }

        do{
            target_pose = move_group->getCurrentPose("iiwa_link_ee");
        }while(target_pose.pose.position.z == 0);

        ROS_INFO("Curren position: %f %f %f.", target_pose.pose.position.x, target_pose.pose.position.y,
                                            target_pose.pose.position.z);
        ROS_INFO("Curren orientation: %f %f %f %f.", target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                                            target_pose.pose.orientation.z, target_pose.pose.orientation.w);                                    

        // Rotate
        if(yaw_rotate != 0 || pitch_rotate != 0 || roll_rotate != 0){
            tf::Quaternion quaternion_;
            double roll,pitch,yaw;
            tf::quaternionMsgToTF(target_pose.pose.orientation, quaternion_);
            tf::Matrix3x3(quaternion_).getRPY(roll, pitch, yaw);

            ROS_INFO("raw roll pitch yaw: %f %f %f.", roll, pitch, yaw);

            yaw += yaw_rotate * yaw_step;
            pitch += pitch_rotate * pitch_step;
            roll += roll_rotate * roll_step;

            ROS_INFO("modify roll pitch yaw: %f %f %f.", roll, pitch, yaw);

            target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        }
        
        // Translate
        target_pose.pose.position.x += x_move*move_step;
        target_pose.pose.position.y += y_move*move_step; 
        target_pose.pose.position.z += z_move*lift_step;

        ROS_INFO("Planning to the position: %f %f %f.", target_pose.pose.position.x, target_pose.pose.position.y,
                                            target_pose.pose.position.z);
        ROS_INFO("Planning to the orientation: %f %f %f %f.", target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                                            target_pose.pose.orientation.z, target_pose.pose.orientation.w); 
        
        setPose(target_pose, frame_id, target_pose.pose.position.x, target_pose.pose.position.y,target_pose.pose.position.z,
                target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);
        
        move_group->setPoseTarget(target_pose);
        evaluate_plan(*move_group);

        execute_in = true;
    }

    //   |  left rocker   |       |  right rocker  |      |right rocker|
    //   |   0       1    |   2   |   3       4    |   5  |      6     |  7
    // [ |   0,      0,   |   0,  |   0,      0,   |   0, |      0,    |  0    ]
    //   |  l  r    f  b  |       |  l  r    f  b  |      |   [arrow]  |
    //   |  1 -1    1 -1  |       |  1 -1    1 -1  |      |   l1  r-1  |
    void joyCallback(const sensor_msgs::Joy& joy_msg){
        double temp_x = joy_msg.axes[1], temp_y = joy_msg.axes[0];
        double temp_z = joy_msg.buttons[0] - joy_msg.buttons[1];
        double temp_yaw = joy_msg.axes[3], temp_pitch = joy_msg.axes[6], temp_roll = -joy_msg.axes[4];
        int pose_flag = joy_msg.buttons[4] + 10 * joy_msg.buttons[5];
        if(state_judge(temp_x, temp_y, temp_z, temp_yaw, temp_pitch, temp_roll, pose_flag)) return ;
        if(execute_in){
            ROS_INFO("===============================================================");
            ROS_INFO("send the sensor_msgs: %f %f %f %f %f %f %d.", temp_x, temp_y, temp_z, temp_yaw, temp_pitch, temp_roll, pose_flag);
            joy_execute(temp_x, temp_y, temp_z, temp_yaw, temp_pitch, temp_roll, pose_flag);
        }
    }

    bool state_judge(double temp_x, double temp_y, double temp_z, double temp_yaw, double temp_pitch, double temp_roll, int pose_flag){
        if(temp_x != 0)
            return false;
        if(temp_y != 0)
            return false;
        if(temp_z != 0)
            return false;
        if(temp_yaw != 0)
            return false;
        if(temp_pitch != 0)
            return false;
        if(temp_roll != 0)
            return false;

        if(pose_flag != 0 && pose_flag != 11)
            return false;

        return true;
    }

    ~IIwaControl(){
        delete move_group;
        delete visual_tools;
    }

        // DEPRECATED
    // void test2(){
    //     move_group->stop();
    //     char c;
    //     double step = 0.1;
    //     while(ros::ok){
    //         ROS_INFO("Please input a instuction(q to quit): ");
    //         std::cin >> c;            
    //         double y_move=0, x_move=0, z_move=0;

    //         if(c == 'a')
    //             x_move -= step;
    //         else if(c == 'd')
    //             x_move += step;
    //         else if(c == 's')
    //             y_move -= step;
    //         else if(c == 'w')
    //             y_move += step;
    //         else if(c == 'z')
    //             z_move -= step;
    //         else if(c == 'x')
    //             z_move += step;
    //         else if(c == 'q')
    //             return;
    //         else
    //             continue;

    //         std::vector<geometry_msgs::Pose> waypoints;
    //         do{
    //             pose_robot_ = move_group->getCurrentPose("iiwa_link_ee").pose;
    //         }while(pose_robot_.position.z == 0);
    //         waypoints.push_back(pose_robot_);

    //         ROS_INFO("Curren position: %f %f %f.", pose_robot_.position.x, pose_robot_.position.y,
    //                                             pose_robot_.position.z);
            
    //         pose_robot_.position.x += x_move;
    //         pose_robot_.position.y += y_move; 

    //         ROS_INFO("Planning to: %f %f %f.", pose_robot_.position.x, pose_robot_.position.y,
    //                                             pose_robot_.position.z);
    //         waypoints.push_back(pose_robot_);
            
    //         if(waypoints.size() >= 2){   
    //             double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //             ROS_INFO("before exe");
    //             move_group->execute(trajectory);
    //             ROS_INFO("after exe");
    //         }
    //         ROS_INFO("circle..");
    //     }
    // }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    std::string PLANNING_GROUP;
    std::string frame_id;

    // DEPRECATED
    // double jump_threshold;
    // double eef_step;

    double move_step;
    double lift_step;
    double roll_step;
    double pitch_step;
    double yaw_step;
    double planning_time;
    // move_control
    double x_move,y_move,z_move;
    bool execute_in;

    moveit::planning_interface::MoveGroupInterface*  move_group;
    moveit_visual_tools::MoveItVisualTools* visual_tools;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group;
    
    geometry_msgs::PoseStamped target_pose;

    // DEPRECATED
    // geometry_msgs::Pose pose_robot_;
    // moveit_msgs::RobotTrajectory trajectory;

};


#endif
