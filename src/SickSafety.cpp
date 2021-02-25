#include "ros/ros.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <std_msgs/Int8MultiArray.h>

class SickSafety
{
public:
  SickSafety();
private:
  std::string robot_ns, front_field_state_topic, back_field_state_topic;
  double lin_speed_, ang_speed_;
  double norm_lin_speed_, norm_ang_speed_;
  bool f_warn_flag, f_stop_flag;
  bool b_warn_flag, b_stop_flag;

  void loadParams();
  void pubVelocity();
  void setSpeed(double linear_vel_, double angular_vel_);

  void frontFieldCallback(const std_msgs::Int8MultiArrayConstPtr& front_state);
  void backFieldCallback(const std_msgs::Int8MultiArrayConstPtr& back_state);

  ros::Subscriber sub1, sub2;
  ros::Publisher pause_pub;
};

SickSafety::SickSafety()
{
  lin_speed_ = 0.3;
  ang_speed_ = 0.2;
  norm_lin_speed_ = 0.0;
  norm_ang_speed_ = 0.0;
  front_field_state_topic = "protective_fileds1";
  back_field_state_topic  = "protective_fileds2";
  f_warn_flag = false;
  b_warn_flag = false;

  f_stop_flag = false;
  b_stop_flag = false;

  loadParams();
}

void SickSafety::loadParams()
{
  ros::NodeHandle nh_("~");
  robot_ns = ros::this_node::getName();

  nh_.getParam("warn_lin_speed", lin_speed_);
  nh_.getParam("warn_ang_speed", ang_speed_);

  nh_.getParam("front_field_state_topic", front_field_state_topic);
  nh_.getParam("back_field_state_topic", back_field_state_topic);

  nh_.getParam("/move_base/DWAPlannerROS/max_vel_x", norm_lin_speed_);
  nh_.getParam("/move_base/DWAPlannerROS/max_rot_vel", norm_ang_speed_);

  sub1 = nh_.subscribe(front_field_state_topic, 1, &SickSafety::frontFieldCallback, this);
  sub2 = nh_.subscribe(back_field_state_topic, 1, &SickSafety::backFieldCallback, this);

}

void SickSafety::setSpeed(double linear_vel_, double angular_vel_)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  double_param.name = "max_vel_x";
  double_param.value = linear_vel_;
  conf.doubles.push_back(double_param);

  double_param.name = "max_rot_vel";
  double_param.value = angular_vel_;
  conf.doubles.push_back(double_param);

  srv_req.config = conf;  
  ros::service::call("move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

}

void SickSafety::pubVelocity()
{
  if(f_stop_flag || b_stop_flag)
   {
    setSpeed(lin_speed_, ang_speed_);
    //ROS_WARN("CALL from STOP");
    }
  else if (f_warn_flag || b_warn_flag)
   {
     //ROS_WARN("CALL from WARN");
     setSpeed(norm_lin_speed_, norm_ang_speed_);
   }
}

void SickSafety::frontFieldCallback(const std_msgs::Int8MultiArrayConstPtr& front_state)
{
  if (front_state->data[0])//Stop zone breached.
  {
    f_stop_flag = true;
    f_warn_flag = false;
  }

  else if (front_state->data[1])//Warning zone breached.
  {
    f_stop_flag = false;
    f_warn_flag = true;
  }
  pubVelocity();
}

void SickSafety::backFieldCallback(const std_msgs::Int8MultiArrayConstPtr& back_state)
{
  if (back_state->data[0])//Stop zone breached.
  {
    b_stop_flag = true;
    b_warn_flag = false;
  }

  else if (back_state->data[1])//Warning zone breached.
  {
    b_stop_flag = false;
    b_warn_flag = true;
  }
  pubVelocity();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SickSafety");
  SickSafety sick_safety_;

  ros::spin();

  return 0;
}
