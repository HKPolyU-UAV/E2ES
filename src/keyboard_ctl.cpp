#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <mutex>
#include <std_msgs/String.h>
#include <utils/include/all_utils.h>
#include <movement/generalmove.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
//#include <curses.h>
//#include <ncurses.h>

//subscribe local_position/pose
//publish   setpoint/pose
#include <termios.h>

char getch_noblocking()
{
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0)
    ROS_ERROR("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0)
    ROS_ERROR("tcsetattr ICANON");

  if(rv == -1)
    ROS_ERROR("select");
  else if(rv == 0)
    ;
    //ROS_INFO("no_key_pressed");
  else
    read(filedesc, &buff, len );

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
    ROS_ERROR ("tcsetattr ~ICANON");
  return (buff);
}

static std::mutex mtx_states_RW;

#define PI (3.1415926)

using namespace std;

static enum Mission_STATE {
  IDLE,
  TAKEOFF,
  KEYBOARD_CTR,
  LAND,
} mission_state=IDLE;

static enum MANEUVER{
  KB_NONE,
  KB_TAKEOFF,
  KB_LAND,
  KB_FORWARD,
  KB_BACKWARD,
  KB_LEFT,
  KB_RIGHT,
  KB_UP,
  KB_DOWN,
  KB_TURNLEFT,
  KB_TURNRIGHT,
} kb_state=KB_NONE;

static mavros_msgs::State current_state;
static bool local_pose_received = false;
static SE3 latest_pos;
static geometry_msgs::PoseStamped last_published_pose;
static geometry_msgs::PoseStamped publish_pose;
static double h_speed;
static double v_speed;
static double turn_speed;
static double update_rate=20.0;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void local_odom_cb(const geometry_msgs::PoseStampedConstPtr& msg){
  mtx_states_RW.lock();
  local_pose_received = true;
  latest_pos.translation().x()=msg->pose.position.x;
  latest_pos.translation().y()=msg->pose.position.y;
  latest_pos.translation().z()=msg->pose.position.z;
  latest_pos.so3() = SO3(Quaterniond(msg->pose.orientation.w,msg->pose.orientation.x,
                                     msg->pose.orientation.y,msg->pose.orientation.z));
  mtx_states_RW.unlock();
}

geometry_msgs::PoseStamped update_pose_from_se3(SE3 se3)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = se3.translation().x();
  pose.pose.position.y = se3.translation().y();
  pose.pose.position.z = se3.translation().z();
  pose.pose.orientation.x = se3.so3().unit_quaternion().x();
  pose.pose.orientation.y = se3.so3().unit_quaternion().y();
  pose.pose.orientation.z = se3.so3().unit_quaternion().z();
  pose.pose.orientation.w = se3.so3().unit_quaternion().w();
  return pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  //initscr();

  ros::NodeHandle nh("~");

  nh.getParam("velocity_horizon", h_speed);
  nh.getParam("velocity_vertical", v_speed);
  nh.getParam("angular_velocity_turn", turn_speed);
  cout << "horizon speed: " <<  h_speed << endl;
  cout << "vertical speed: " << v_speed << endl;
  cout << "turning speed: " <<  turn_speed << endl;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
      ("/mavros/state", 10, state_cb);
  ros::Subscriber local_odom_sub = nh.subscribe<geometry_msgs::PoseStamped>
      ("/mavros/local_position/pose", 10, local_odom_cb);

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      ("/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("/mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(update_rate);
  //wait for FCU connection
  cout << "Waiting for FCU connection " << endl;
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
    cout << "Waiting for FCU connection " << endl;
  }
  while(local_pose_received==false)
  {
    ros::spinOnce();
    rate.sleep();
    cout << "Waiting for Local Position" << endl;
  }
  //send a few setpoints before starting
  mtx_states_RW.lock();
  geometry_msgs::PoseStamped init_pose = update_pose_from_se3(latest_pos);
  mtx_states_RW.unlock();
  for(int i = 50; ros::ok() && i > 0; --i){
    local_pos_pub.publish(init_pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  cout << "change last_request Arm" << endl;
  ros::Time last_request = ros::Time::now();

  while(ros::ok()){

    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
    {
      if( set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(1.0))){
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
          mission_state = IDLE;
        }
        last_request = ros::Time::now();
      }
    }
    kb_state=KB_NONE;
    //        KB_NONE,
    //        KB_TAKEOFF,
    //        KB_LAND,
    //        KB_FORWARD,
    //        KB_BACKWARD,
    //        KB_LEFT,
    //        KB_RIGHT,
    //        KB_UP,
    //        KB_DOWN,
    //        KB_TURNLEFT,
    //        KB_TURNRIGHT,
    //timeout(50);
    fseek(stdin,0,SEEK_END);
    int c = getch_noblocking();
    fseek(stdin,0,SEEK_END);
    //flushinp();

    if (c == '1')
    {
      kb_state=KB_TAKEOFF;
      cout << "kb_takeoff";
    }
    if (c == '2')
    {
      kb_state=KB_LAND;
      cout << "kb_land";
    }
    if (c == 'w' || c == 'W')
    {
      kb_state=KB_UP;
      cout << "kb_up";
    }
    if (c == 's' || c == 'S')
    {
      kb_state=KB_DOWN;
      cout << "kb_down";
    }
    if (c == 'a' || c == 'A')
    {
      kb_state=KB_TURNLEFT;
      cout << "kb_turn_left";
    }
    if (c == 'd' || c == 'D')
    {
      kb_state=KB_TURNRIGHT;
      cout << "kb_turn_right";
    }
    if (c == 'y' || c == 'Y')
    {
      kb_state=KB_FORWARD;
      cout << "kb_forward";
    }
    if (c == 'h' || c == 'H')
    {
      kb_state=KB_BACKWARD;
      cout << "kb_backward";
    }
    if (c == 'g' || c == 'G')
    {
      kb_state=KB_LEFT;
      cout << "kb_left";
    }
    if (c == 'j' || c == 'J')
    {
      kb_state=KB_RIGHT;
      cout << "kb_right";
    }
    /*takeoff*****************************************************/
    //PLEASE DEFINE THE LANDING PARAMETER HERE
    if(mission_state==IDLE)
    {
      if(kb_state==KB_TAKEOFF)
      {
        mission_state = TAKEOFF;
      }
    }
    if(mission_state==TAKEOFF)
    {
      Vec3 rpy=Q2rpy(Quaterniond(latest_pos.so3().unit_quaternion().w(),
                                 latest_pos.so3().unit_quaternion().x(),
                                 latest_pos.so3().unit_quaternion().y(),
                                 latest_pos.so3().unit_quaternion().z()));
      static generalMove takeoff(ros::Time::now().toSec(),
                                 latest_pos.translation().x(),
                                 latest_pos.translation().y(),
                                 latest_pos.translation().z(),
                                 rpy(2),
                                 latest_pos.translation().x(),
                                 latest_pos.translation().y(),
                                 1.5,
                                 rpy(2),
                                 8);
      takeoff.getPose(ros::Time::now().toSec(),publish_pose);
      cout << publish_pose.pose.position.x << "," << publish_pose.pose.position.y << "," << publish_pose.pose.position.z << endl;
      if(takeoff.finished())
      {
        cout << "Takeoff finished" << endl;
        mission_state = KEYBOARD_CTR;
        last_request = ros::Time::now();
      }
    }
    if(mission_state==KEYBOARD_CTR)
    {
      Vec3 rpy_last;
      Vec3 xyz_last;
      Quaterniond q_last(last_published_pose.pose.orientation.w,
                         last_published_pose.pose.orientation.x,
                         last_published_pose.pose.orientation.y,
                         last_published_pose.pose.orientation.z);
      rpy_last=Q2rpy(q_last);
      xyz_last = Vec3(last_published_pose.pose.position.x,
                      last_published_pose.pose.position.y,
                      last_published_pose.pose.position.z);
      publish_pose = last_published_pose;
      Vec3 rpy_new = rpy_last;
      Vec3 xyz_new = xyz_last;
      switch(kb_state)
      {
      case KB_FORWARD:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        xyz_new(0) += cos(rpy_last(2))*(h_speed/update_rate);
        xyz_new(1) += sin(rpy_last(2))*(h_speed/update_rate);
        break;
      case KB_BACKWARD:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        xyz_new(0) -= cos(rpy_last(2))*(h_speed/update_rate);
        xyz_new(1) -= sin(rpy_last(2))*(h_speed/update_rate);
        break;
      case KB_LEFT:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        xyz_new(0) += cos(rpy_last(2)+PI/2.0)*(h_speed/update_rate);
        xyz_new(1) += sin(rpy_last(2)+PI/2.0)*(h_speed/update_rate);
        break;
      case KB_RIGHT:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        xyz_new(0) += cos(rpy_last(2)-PI/2.0)*(h_speed/update_rate);
        xyz_new(1) += sin(rpy_last(2)-PI/2.0)*(h_speed/update_rate);
        break;
      case KB_UP:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        xyz_new(2) += v_speed/update_rate;
        break;
      case KB_DOWN:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        xyz_new(2) -= v_speed/update_rate;
        break;
      case KB_TURNLEFT:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        rpy_new(2) += turn_speed/update_rate;
        break;
      case KB_TURNRIGHT:
        rpy_new = Vec3(0,0,rpy_last(2));
        xyz_new = xyz_last;
        rpy_new(2) -= turn_speed/update_rate;
        break;
      case KB_LAND:
        rpy_new = rpy_last;
        xyz_new = xyz_last;
        mission_state=LAND;
        break;
      }
      Quaterniond q_new=rpy2Q(rpy_new);
      publish_pose.pose.position.x=xyz_new(0);
      publish_pose.pose.position.y=xyz_new(1);
      publish_pose.pose.position.z=xyz_new(2);
      publish_pose.pose.orientation.w = q_new.w();
      publish_pose.pose.orientation.x = q_new.x();
      publish_pose.pose.orientation.y = q_new.y();
      publish_pose.pose.orientation.z = q_new.z();
    }
    if(mission_state==LAND)
    {
      Vec3 rpy=Q2rpy(Quaterniond(latest_pos.so3().unit_quaternion().w(),
                                 latest_pos.so3().unit_quaternion().x(),
                                 latest_pos.so3().unit_quaternion().y(),
                                 latest_pos.so3().unit_quaternion().z()));
      static generalMove land(ros::Time::now().toSec(),
                              latest_pos.translation().x(),
                              latest_pos.translation().y(),
                              latest_pos.translation().z(),
                              rpy(2),
                              latest_pos.translation().x(),
                              latest_pos.translation().y(),
                              0,
                              rpy(2),
                              latest_pos.translation().z()/v_speed);
      land.getPose(ros::Time::now().toSec(),publish_pose);
      cout << publish_pose.pose.position.x << "," << publish_pose.pose.position.y << "," << publish_pose.pose.position.z << endl;
      if(land.finished())
      {
        cout << "land finished" << endl;
        mission_state = KEYBOARD_CTR;
        last_request = ros::Time::now();
      }
    }
    //publish_pose = update_pose_from_se3(latest_pos);
    if(kb_state!=KB_NONE)
    {
      cout  << publish_pose.pose.position.x << "," << publish_pose.pose.position.y << "," << publish_pose.pose.position.z << endl;
    }
    local_pos_pub.publish(publish_pose);
    last_published_pose = publish_pose;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
