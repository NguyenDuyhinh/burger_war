// 本プログラムは
// http://wiki.ros.org/ja/navigation/Tutorials/SendingSimpleGoals
// を元に作成しています。
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose {
  double x;
  double y;
  double yaw;
};

int main(int argc, char** argv){
  MyPose way_point[] = {
    { 0.010,  0.007, -0.666},
    { 0.835, -0.589,  0.016},
    { 0.848, -0.445,  1.564},
    { 1.036,  0.168,  0.145},
    { 1.036,  0.168, -0.757},
    { 1.923, -0.494,  0.200}, 
    { 1.923, -0.494,  1.578}, 
    { 1.923, -0.494, -3.160},
    { 1.923, -0.494,  0.664},
    { 2.820,  0.412, -2.888},
    { 2.820,  0.412,  2.537},
    { 1.864,  1.042, -3.123},
    { 1.868,  1.042, -1.514},
    { 1.864,  1.042,  0.054},
    //{ 8.091, -3.0, -1.57},
    //{ 12.195, -3.7, 0.1},//0.987
    //{ 12.630, -3.335, 1.65},//1.586

    {999, 999, 999}};

  ros::init(argc, argv, "wp_navigation");

  // アクションクライアンを作成。1番目の引数は接続するアクションサーバー名。
  // ２番目の引数はtrueならスレッドを自動的に回す(ros::spin()。
  MoveBaseClient ac("move_base", true);

  // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
  // この例では５秒間待つ(ブロックされる)
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("The server comes up");
  move_base_msgs::MoveBaseGoal goal;
  // map(地図)座標系
  goal.target_pose.header.frame_id = "map";
  // 現在時刻                                                                       
  goal.target_pose.header.stamp = ros::Time::now();


  int i = 0;
  while (ros::ok()) {
    // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
    goal.target_pose.pose.position.x =  way_point[i].x;
    goal.target_pose.pose.position.y =  way_point[i].y;

    if (goal.target_pose.pose.position.x == 999) break;

    goal.target_pose.pose.orientation 
      = tf::createQuaternionMsgFromYaw(way_point[i].yaw);

    ROS_INFO("Sending goal: No.%d", i+1);
    ROS_INFO("X=%f Y=%f th=%f", way_point[i].x, way_point[i].y, way_point[i].yaw);
    
    // サーバーにgoalを送信
    ac.sendGoal(goal);

    // 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
    bool succeeded = ac.waitForResult(ros::Duration(10.0));

    // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
    actionlib::SimpleClientGoalState state = ac.getState();

    if(succeeded) {
      ROS_INFO("Succeeded: No.%d (%s)",i+1, state.toString().c_str());
    }
    else {
      ROS_INFO("Failed: No.%d (%s)",i+1, state.toString().c_str());
    }
    i++;
  }
  return 0;
}
