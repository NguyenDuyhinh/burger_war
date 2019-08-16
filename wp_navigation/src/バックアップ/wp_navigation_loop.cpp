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
    //{ 0.696, 4.584, 3.115},
   // { 2.476, 0.819, -1.571},
    //{ 0.0, 0.0, -3.2}, //3月22日のデモ用
    //{ 2.600, 5.40,-1.641},　//3月22日のデモ用
    //{ 0.0, 0.0, -3.2},//実験用
    //{ 2.681, 4.461,1.571},// 実験用
    //{ 8.091, -3.329, -1.57},
    { 12.880, -3.335, 1.65},//1.586
    //{ 0.0, 0.0, -3.2},
    //{ 2.681, 4.461,1.571}, 
    //{ 0.0, 0.0, -3.2},
    //{ 2.681, 4.461,1.571}, 
    //{ 0.0, 0.0, -3.2},
    //{ 2.681, 4.461,1.571}, 
    //{ 0.0, 0.0, -3.2},
    //{ 2.681, 4.461,1.571}, 
    //{ 0.0, 0.0, 0.0 * M_PI},
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
    bool succeeded = ac.waitForResult(ros::Duration(60.0));

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
