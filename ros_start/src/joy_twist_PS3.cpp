#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

  enum {
   IDX_SELECT = 0,
   IDX_JOY_L = 1,
   IDX_JOY_R = 2,
   IDX_START = 3,
   IDX_UP = 4,
   IDX_RIGHT = 5,
   IDX_DOWN = 6,
   IDX_LEFT = 7,
   IDX_L2 = 8,
   IDX_R2 = 9,
   IDX_L1 = 10,
   IDX_R1 = 11,
   IDX_TRIANGLE = 12,
   IDX_CIRCLE = 13,
   IDX_X = 14,
   IDX_RECTANGLE = 15,
   IDX_MAX,
  };

  const char cmd[][64] = {
     /* セレクト    */ "",
     /* 左スティック */ "",
     /* 右スティック */ "",
     /* スタート    */ "rosservice call /whill_next_control/move_again &",
     /* 上        */ "",
     /* 右        */ "",
     /* 下        */ "rosservice call /move_base/clear_costmaps &",
     /* 左        */ "",
     /* L2        */ "",
     /* R2        */ "",
     /* L1        */ "",
     /* R1        */ "",
     /* 三角      */ "rosservice call /whill_next_control/move_start 4 &",
     /* マル      */ "rosservice call /whill_next_control/move_start 5 &",
     /* バツ      */ "rosservice call /whill_next_control/move_stop &",
     /* 四角      */ "rosservice call /whill_next_control/set_wp 0 &",
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
    if (joy_msg.buttons[0] == 1 && push_flag[0] == 0) {
        mode ++ ; 
        if(mode == 2){mode = 0;}
        push_time_[0] = ros::Time::now();
        push_flag[0] = 1;
    }
    under_time_ = ros::Time::now() - push_time_[0];
    if ( under_time_ > ros::Duration(0.5)){push_flag[0] = 0;}
    //ROS_INFO("mode = %d",mode);

    int err;

    // 押されたボタンのうち、一番小さい番号を取得する（L1と左スティックは除外）
    int index;
    for ( index = 0; index < IDX_MAX; index++ ) {
        if ( (index == IDX_JOY_L) || (index == IDX_L1) || (index == IDX_R1)) {
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
        if ( diff_time > ros::Duration(0.5) && joy_msg.buttons[11] == 1){
            ROS_INFO("pressed index %d", index);
            if ( strlen(cmd[index]) != 0 ) {
                err = system(cmd[index]);
            }
            push_time_[index] = ros::Time::now();
        }
    }

    if(mode == 0){
       if (joy_msg.buttons[10] == 1)
       {
          geometry_msgs::Twist twist;
          if (joy_msg.buttons[11] == 1) {
          twist.linear.x = joy_msg.axes[1] * 2.0;
          twist.angular.z = joy_msg.axes[0] * 2.0;}
          else{
          twist.linear.x = joy_msg.axes[1] * 1.0;
          twist.angular.z = joy_msg.axes[0] * 1.0;}
          twist_pub_.publish(twist);
       }
     }
     else{
       if (joy_msg.buttons[10] == 1)
       {
          geometry_msgs::Twist twist;
          if (joy_msg.buttons[11] == 1) { 
          twist.linear.x = joy_msg.axes[1] * 2.0;
          twist.angular.z = joy_msg.axes[2] * 2.0;}
          else{
          twist.linear.x = joy_msg.axes[1] * 1.0;
          twist.angular.z = joy_msg.axes[2] * 1.0;}
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
