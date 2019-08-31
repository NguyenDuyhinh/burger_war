#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define GAIN_CHASE -0.01
#define PNT_START_CHASE 9

int TRACE_AREA_SIZE;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose
{
  double x;
  double y;
  double yaw;
  bool is_checkpoint;
  double time;
};

MyPose way_point[] = {
   // {-1.150,  0.000, -0.785, false, 0.0}, // 右手前ブロック、手前側の点を取る -0.087
    {-0.788, -0.496, -0.760, false, 0.0},
    {-0.812, -0.464, -0.000, false, 0.0},
    {-0.812, -0.464, 1.546, false, 0.0}, // 左手前ブロック、手前側の点を取る
    {-0.812, 0.500, 1.546, false, 0.0},
    {-0.812, 0.500, 0.000, false, 0.0},
    {-0.812, 0.500, -1.596, false, 0.0}, // 中央ブロック、手前側の点を取る
    {-0.812, -0.100, -1.596, false, 0.0},
    {-0.812, -0.100, 0.000, false, 0.0},
    {-0.408, 0.000, 0.000, false, 0.0},
    {-0.408, 0.000, -0.829, false, 0.0}, // 右奥ブロック、奥側の点を取る
    {0.062, -0.590, -0.815, false, 0.0},
    {0.062, -0.590, 0.000, false, 0.0},
    {0.062, -0.590, 1.546, false, 0.0},  // 右奥ブロック、中心の点を取る
    {0.062, -0.590, -3.140, false, 0.0}, // 右奥ブロック、手前側の点を取る
    {0.062, -0.590, 2.302, false, 0.0},
    {-0.600, 0.050, 1.570, false, 0.0}, // 正面、横向き、経過点
    {-0.600, 0.050, 0.756, false, 0.0}, // 正面、左上向き、経過点
    {0.095, 0.585, 0.836, false, 0.0},  // 左奥ブロック、奥側の点を取る
    {0.019, 0.572, 0.000, false, 0.0},
    {0.019, 0.572, -1.571, false, 0.0}, // 左奥ブロック、中心の点を取る
    {0.019, 0.572, -3.140, false, 0.0}, // 左奥ブロック、手前側の点を取る
    {0.019, 0.572, -0.741, false, 0.0}, // 敵ブロック、手前側の点を取る
    {0.597, -0.057, -0.780, false, 0.0},
    {0.553, 0.006, 3.142, false, 0.0},
    {0.553, 0.006, -2.291, false, 0.0}, // 右奥ブロック、中心の点を取る
    {-0.029, -0.611, -2.350, false, 0.0},
    {0.022, -0.566, 1.576, false, 0.0},
    {0.015, -0.515, 0.000, false, 0.0},  // 右奥ブロック、奥側の点を取る
    {0.015, -0.515, -3.140, false, 0.0}, // 右奥ブロック、手前側の点を取る
                                         //{0.2,    0.5, 0, true, 1.0},           // 左奥ブロック、手前側の点を取る
                                         //{0,      0.5, -1.57, true, 2.0},       // 中央ブロック、左側の点を取る
    {999, 999, 999}};

static const std::string OPENCV_WINDOW = "Image window";

class RoboCtrl
{
private:
  enum EState
  {
    STATE_IDLE = 0,
    STATE_WAYPOINT = 1,
    STATE_CHASE = 2,
    STATE_BACK = 3,
    STATE_INITIAL_GO = 4
  };

public:
  EState m_state;
  RoboCtrl() : it_(node), ac("move_base", true)
  {
    ros::NodeHandle node;
    //購読するtopic名
    //odom_sub_ = node.subscribe("odom", 1, &RoboCtrl::odomCallback, this);
    image_sub_ = it_.subscribe("image_raw", 1, &RoboCtrl::imageCb, this);

    //配布するtopic名
    //twist_pub_ = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    twist_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //内部変数初期化
    m_frontspeed = 0.5;
    m_turnspeed = 0.0;
    m_diffPos = 0.0;

    // まず最初はウェイポイント
    // m_state = STATE_WAYPOINT;
    m_state = STATE_INITIAL_GO;
    time_start = ros::Time::now();

    MoveBaseClient ac("/move_base", true);
    initWaypoint();
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~RoboCtrl()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void initWaypoint()
  {
    // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
    // この例では５秒間待つ(ブロックされる)
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("The server comes up");
    m_isSent = false; //?
    m_destPnt = 0;    //?
  }

  void moveRobo()
  {
    //速度データ型宣言
    geometry_msgs::Twist twist;

    checkWaypoint();
    getCurrentPosition();

    // 壁際から抜け出すために最初に直進
    ROS_INFO("robo state: %d", m_state);
    if (m_state == STATE_INITIAL_GO)
    {
      m_frontspeed = 0.2;
      m_turnspeed = 0;
      ROS_INFO("time duration: %f", (ros::Time::now() - time_start).toSec());
      if (3.0 < (ros::Time::now() - time_start).toSec())
      {
        m_state = STATE_WAYPOINT;
      }
    }

    // STATE_CHASEの場合 P制御で追いかける
    if (m_state == STATE_CHASE)
    {
      ros::Duration tick = ros::Time::now() - m_timechasestr;
      double tickdbl = tick.toSec();

      m_frontspeed = 0.2;
      m_turnspeed = m_diffPos * GAIN_CHASE;
      ROS_INFO("CHASE PHASE(1) %f", tickdbl);

      /*
               if( tickdbl <= 8.0 ){
                  m_frontspeed = 0.1;
                  m_turnspeed = m_diffPos * GAIN_CHASE;
                  ROS_INFO("CHASE PHASE(1) %f", tickdbl);
               } else if( tickdbl <= 16.0 ) {
                  m_frontspeed = 0.1;
                  m_turnspeed = m_diffPos * GAIN_CHASE;
                  ROS_INFO("CHASE PHASE(2) %f", tickdbl);
               } else if( tickdbl > 16.0 ) {
                  m_frontspeed = 0.1;
                  m_turnspeed = -m_diffPos * GAIN_CHASE;
                  ROS_INFO("CHASE PHASE(3) %f", tickdbl);
	       }*/
    }

    if (m_state == STATE_BACK)
    {
      ros::Duration tick = ros::Time::now() - time_back;
      double tickdbl = tick.toSec();
      if (tickdbl <= way_point[m_destPnt - 1].time)
      {
        m_frontspeed = -0.2;
        m_turnspeed = 0.0;
      }
      else if (tickdbl > way_point[m_destPnt - 1].time)
      {
        m_frontspeed = 0.0;
        m_frontspeed = 0.0;
        m_state = STATE_WAYPOINT;
      }
    }

    // ウェイポイント終わったらSTATE_IDLEにしてその場で回る
    if (m_state == STATE_IDLE)
    {
      ros::Duration tick = ros::Time::now() - time_idle;
      double tickdbl = tick.toSec();
      if (tickdbl <= 1.0)
      {
        m_frontspeed = 0.0;
        m_turnspeed = -2.0;
      }
      else if (tickdbl <= 3.0)
      {
        m_frontspeed = 0.0;
        m_turnspeed = 2.0;
      }
      else if (tickdbl <= 5.0)
      {
        m_frontspeed = 0.0;
        m_turnspeed = -2.0;
      }
      else
      {
        time_idle = ros::Time::now() - ros::Duration(1.0);
      }
    }

    ROS_INFO("NOW %d", m_state);
    ROS_INFO("diff %f", m_diffPos);

    //ROS速度データに内部関数値を代入
    if (m_state == STATE_CHASE || m_state == STATE_IDLE || m_state == STATE_BACK || m_state == STATE_INITIAL_GO)
    {
      twist.linear.x = m_frontspeed;
      twist.angular.z = m_turnspeed;
      twist_pub_.publish(twist);
    }
  }

  void checkWaypoint()
  {
    // ウェイポイント送信済みか？
    if (m_isSent)
    {
      // ウェイポイント送信済みなら到着結果の確認をする
      // 追跡モード中はm_isSentはfalseなので確認しない
      bool isSucceeded = ac.waitForResult(ros::Duration(0.5));
      // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
      actionlib::SimpleClientGoalState state = ac.getState();

      // 到着済みか?
      if (isSucceeded)
      {
        // 到着済みなら次の点を送信する
        m_isSent = false;
        ROS_INFO("WP Reached: No.%d (%s)", m_destPnt + 1, state.toString().c_str());

        if (state == actionlib::SimpleClientGoalState::ABORTED)
        {

          ROS_INFO("NO MORE WAYPOINT!!!");
          m_state = STATE_IDLE;
          time_idle = ros::Time::now();
        }
        else
        {
          time_back = ros::Time::now();
          ros::Duration(0.5).sleep();

          if (way_point[m_destPnt].is_checkpoint)
          {
            ROS_INFO("BACK START");
            m_state = STATE_BACK;
          }
          ++m_destPnt;
        }

        ros::Duration(1.0).sleep(); //次のwaypoint送る前に安定タイマー
                                    //sendWaypoint(++m_destPnt);
      }
      else
      {
        // 到着してないなら何もしない
        ROS_INFO("WP not reached: No.%d (%s)", m_destPnt + 1, state.toString().c_str());
      }
    }
    else
    {
      // ウェイポイント送信済みでない場合
      if (m_state == STATE_WAYPOINT)
      {
        sendWaypoint(m_destPnt);
      }
    }
  }

  void sendWaypoint(int n_ptr)
  {
    // STATE_WAYPOINTの場合のみ呼ばれる
    // ウェイポイントの更新 or 初回時 or STATE_CHASEからの復帰
    move_base_msgs::MoveBaseGoal goal;
    // map(地図)座標系
    goal.target_pose.header.frame_id = "/map";
    // 現在時刻
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = way_point[n_ptr].x;
    goal.target_pose.pose.position.y = way_point[n_ptr].y;

    // 最終ウェイポイントに達したらSTATE_IDLEに遷移して抜ける。
    if (goal.target_pose.pose.position.x == 999)
    {
      m_state = STATE_IDLE;
      time_idle = ros::Time::now();
      return;
    }

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[n_ptr].yaw);

    ROS_INFO("Sending goal: No.%d", n_ptr + 1);
    ROS_INFO("X=%f Y=%f th=%f", way_point[n_ptr].x, way_point[n_ptr].y, way_point[n_ptr].yaw);

    // サーバーにgoalを送信
    ac.sendGoal(goal);

    m_isSent = true;
  }

  void cancelWayPoint()
  {
    // ウェイポイントのキャンセル。STATE_CHASEへの移行時に呼ばれる
    ac.cancelAllGoals();
    ROS_INFO("WAYPOINT CANCELED. START CHASING!!");

    // STATE_WAYPOINT復帰時にウェイポイントを送り直すようにしておく
    m_isSent = false;
  }

  void odomCallback(const nav_msgs::Odometry &odom)
  {
    return;
  }

  bool getCurrentPosition()
  {

    double x, y, yaw;
    try
    {
      tf::StampedTransform trans;
      tfl_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.5));
      tfl_.lookupTransform("/map", "/base_link", ros::Time(0), trans);

      x = trans.getOrigin().x();
      y = trans.getOrigin().y();
      yaw = tf::getYaw(trans.getRotation());
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
      return false;
    }

    ROS_INFO("Now Pos %0.3f %0.3f %0.3f", x, y, yaw);

    return true;
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    const int centerpnt = msg->width / 2;
    const int range = 20; //10;
    static EState laststate = m_state;
    double area_min = 0;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(60 - range, 50, 50), cv::Scalar(60 + range, 255, 255), mask); // 色検出でマスク画像の作成
    //cv::bitwise_and(cv_ptr->image,mask,image);

    cv::Moments mu = cv::moments(mask, false);
    cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

    double area = mu.m00;
    int x = mu.m10 / mu.m00;
    int y = mu.m01 / mu.m00;
    ROS_INFO("AREA = %f", area);

    if (m_destPnt >= PNT_START_CHASE)
    {
      area_min = 20000;
    }
    else
    {
      area_min = TRACE_AREA_SIZE;
    }

    // 敵が見つかったら追跡する
    static ros::Time lastEnmDetectTime;
    if ((x >= 0) && (x <= msg->width) && (area > area_min))
    {
      ROS_WARN("ENEMY DETECTED");

      lastEnmDetectTime = ros::Time::now();
      m_diffPos = x - centerpnt;

      // STATE_CHASEに入る前の状態を保存
      switch (m_state)
      {

      case STATE_WAYPOINT:
        cancelWayPoint();
        break;

      case STATE_IDLE:
        laststate = m_state;
        m_timechasestr = ros::Time::now();
        break;

      default:
        break;
      }

      // STATE_CHASEに遷移
      m_state = STATE_CHASE;

      // 敵を見失う or そもそも敵を見つけていない場合
    }
    else
    {
      if (1.5 < (ros::Time::now() - lastEnmDetectTime).toSec())
      {
        m_diffPos = 0;
        // STATE_CHASEに入る前の状態を戻す
        if (m_state == STATE_CHASE)
        {
          m_state = laststate;
        }
      }
    }

    //ROS_INFO("obj x=%d y=%d",x,y);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, mask);
    cv::waitKey(3);
  }

private:
  ros::NodeHandle node;
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber Laser_sub_;
  ros::Publisher twist_pub_;

  //ROS時間
  ros::Time push_time_;
  //ROS変化秒
  ros::Duration under_time_;

  cv::Mat hsv;
  cv::Mat mask;
  cv::Mat image;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;


  // ロボット制御用
  double m_diffPos;
  double m_frontspeed;
  double m_turnspeed;
  ros::Time m_timechasestr;
  ros::Time time_back;
  ros::Time time_idle;
  ros::Time time_start;

  // ウェイポイント制御用
  int m_destPnt;
  bool m_isSent;
  MoveBaseClient ac;

  //現在位置取得用
  tf::TransformListener tfl_;
};

int main(int argc, char **argv)
{
  //ROSのノード初期化
  ros::init(argc, argv, "robo_ctrl");
  RoboCtrl robo_ctrl;
  ros::Rate r(20);

  ros::param::get("~trace_area_size", TRACE_AREA_SIZE);
  ROS_INFO("TRACE_AREA_SIZE: %d \n\r", TRACE_AREA_SIZE);

  while (ros::ok())
  {
    robo_ctrl.moveRobo();
    ros::spinOnce();
    r.sleep();
  }
}
