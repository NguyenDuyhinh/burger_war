#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Head Teleop
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

struct MyPose {
  double x;
  double y;
};

int main(int argc, char** argv)
{
      MyPose point[] = {
    { 1.0, 0.0},
    //{999, 999},
    { -1.2, -1.2},
    { 1.2, 1.2,},
    { -1.2, 1.2}, 
    {999, 999}};


    //std::string action_name = "head_controller/follow_joint_trajectory";
    std::string head_pan_joint_, head_tilt_joint_;
    ros::init(argc, argv, "head_navigation");
  
    client_t client("head_controller/follow_joint_trajectory", true);

    while(!client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the head_controller/follow_joint_trajectory action server to come up");
    }

    ROS_INFO("The server comes up");

    control_msgs::FollowJointTrajectoryGoal goal;

    head_pan_joint_ = "head_pan_joint";
    head_tilt_joint_ = "head_tilt_joint";

    goal.trajectory.joint_names.push_back(head_pan_joint_);
    goal.trajectory.joint_names.push_back(head_tilt_joint_);

    int i;
    //double step = 5.0;
      
    while (ros::ok()){  
    
    if ((point[i].x) == 999) break;
      
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(point[i].x);
    p.positions.push_back(point[i].y);
    p.velocities.push_back(1.0);
    p.velocities.push_back(1.0);
    ROS_INFO("The server comes up2");
    p.time_from_start = ros::Duration(10.0);
    //ROS_INFO("The server comes up3");
    goal.trajectory.points.push_back(p);
    //ROS_INFO("The server comes up4");
    goal.goal_time_tolerance = ros::Duration(0.0);
    

    // サーバーにgoalを送信
    client.sendGoal(goal);
  
    client.waitForResult();
    
    //sleep(3.0);
 
    // 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
    bool succeeded = client.waitForResult(ros::Duration(5.0));

    // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
    actionlib::SimpleClientGoalState state = client.getState();

    if(succeeded) {
      ROS_INFO("Succeeded: No.%d (%s)",i, state.toString().c_str());
    }
    else {
      ROS_INFO("Failed: No.%d (%s)",i, state.toString().c_str());
    }
    
    i++;
  
    }

    ROS_INFO("finish");
    
    //ros::Duration(10.0);

    
    
}



