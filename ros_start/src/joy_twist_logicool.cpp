#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

  enum {
   IDX_A = 0,
   IDX_B = 1,
   IDX_X = 2,
   IDX_Y = 3,
   IDX_LB = 4,
   IDX_RB = 5,
   IDX_BACK = 6,
   IDX_START = 7,
   IDX_Logicool = 8,
   IDX_L = 9,  //アナログスティック
   IDX_R = 10, //アナログスティック
   IDX_MAX,
  };

  const char cmd[][64] = {
     /* A */ "rosservice call /whill_next_control/move_stop &",
     /* B */ "rosservice call /whill_next_control/move_start 11 &",
     /* X */ "rosservice call /whill_next_control/set_wp 0 &",
     /* Y */ "rosservice call /whill_next_control/move_start 30 &",
     /*LB */ "",
     /*RB */ "",
     /*BACK*/ "rosservice call /whill_next_control/set_wp 11 &",
     /*START*/ "rosservice call /whill_next_control/move_again &",
     /*Logicool*/ "",
     /* L */ "",
     /* R */ "",
  };


class JoyTwist
{
public:
  JoyTwist()
  {
    ros::NodeHandle node;
    joy_sub_ = node.subscribe("joy", 1, &JoyTwist::joyCallback, this);
    twist_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    mode = 0;
    for ( int i = 0; i < IDX_MAX; i++ ) {
      push_flag[i] = 0;
    }
  }
  
  void joyCallback(const sensor_msgs::Joy &joy_msg)
  {
//ROS_INFO("joy called");
    if (joy_msg.buttons[IDX_Logicool] == 1 && push_flag[0] == 0) {
        mode ++ ; 
        if(mode == 2){mode = 0;}
        push_time_[0] = ros::Time::now();
        push_flag[0] = 1;
    }
    under_time_ = ros::Time::now() - push_time_[0];
    if ( under_time_ > ros::Duration(0.5)){push_flag[0] = 0;}
    //ROS_INFO("mode = %d",mode);

    int err;

    // 押されたボタンのうち、一番小さい番号を取得する（LBとRBは除外）
    int index;
    for ( index = 0; index < IDX_MAX; index++ ) {
        if ( (index == IDX_LB) || (index == IDX_RB)) {
            continue;
        }
        if ( joy_msg.buttons[index] == 1 ) {
            break;
        }
    }
    // 番号が最大の場合は、L1、左スティックが押されているので速度を出す処理だけさせる
    if ( index != IDX_MAX ) {
        // 時間が0.5秒経過していれば、コマンドを起動する
        ros::Duration diff_time;
        diff_time = ros::Time::now() - push_time_[index];
        if ( diff_time > ros::Duration(0.5) && joy_msg.buttons[IDX_RB] == 1){
            ROS_INFO("pressed index %d", index);
            if ( strlen(cmd[index]) != 0 ) {
                err = system(cmd[index]);
            }
            push_time_[index] = ros::Time::now();
        }
    }

    if(mode == 0){
       if (joy_msg.buttons[IDX_LB] == 1)
       {
          geometry_msgs::Twist twist;
          if (joy_msg.buttons[IDX_RB] == 1) {
          twist.linear.x = joy_msg.axes[1] * 2.0;
          twist.angular.z = joy_msg.axes[0] * 2.0;}
          else{
          twist.linear.x = joy_msg.axes[1] * 1.0;
          twist.angular.z = joy_msg.axes[0] * 1.0;}
          twist_pub_.publish(twist);
       }
     }
     else{
       if (joy_msg.buttons[IDX_LB] == 1)
       {
          geometry_msgs::Twist twist;
          if (joy_msg.buttons[IDX_RB] == 1) { 
          twist.linear.x = joy_msg.axes[1] * 2.0;
          twist.angular.z = joy_msg.axes[3] * 2.0;}
          else{
          twist.linear.x = joy_msg.axes[1] * 1.0;
          twist.angular.z = joy_msg.axes[3] * 1.0;}
          twist_pub_.publish(twist);
       }
     }
  }
private:
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::Time push_time_[IDX_MAX];
  ros::Duration under_time_;
  int mode;
  int push_flag[IDX_MAX];
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_twist");
  JoyTwist joy_twist;
  ros::spin();
}
