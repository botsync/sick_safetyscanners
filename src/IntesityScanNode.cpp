#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8MultiArray.h>
#include <sick_safetyscanners/ExtendedLaserScanMsg.h>
#include <sick_safetyscanners/RawMicroScanDataMsg.h>

std::string publish_topic, input_topic, raw_topic;
ros::Publisher intensity_pub, protective_fields;
//ros::Subscriber laser_sub;

void loadParams(){
  ros::NodeHandle nh_("~");

  nh_.getParam("destination_topic", publish_topic);
  nh_.getParam("input_topic", input_topic);
  nh_.getParam("raw_data_topic", raw_topic);
}

void laserScanCallback(const sick_safetyscanners::ExtendedLaserScanMsgConstPtr &laser_msg){
  sensor_msgs::LaserScan intensity_scan;

  intensity_scan.header.frame_id = laser_msg->laser_scan.header.frame_id;
  intensity_scan.angle_min = laser_msg->laser_scan.angle_min;
  intensity_scan.angle_max = laser_msg->laser_scan.angle_max;
  intensity_scan.angle_increment = laser_msg->laser_scan.angle_increment;
  intensity_scan.time_increment = laser_msg->laser_scan.time_increment;
  intensity_scan.scan_time = laser_msg->laser_scan.scan_time;
  intensity_scan.range_min = laser_msg->laser_scan.range_min;
  intensity_scan.range_max = laser_msg->laser_scan.range_max;

  int data_size_ = laser_msg->laser_scan.ranges.size();

  intensity_scan.ranges.resize(data_size_);
  intensity_scan.intensities.resize(data_size_);

  for (int i=0; i<data_size_; i++)
    if (laser_msg->reflektor_status[i]){
      intensity_scan.ranges[i] = laser_msg->laser_scan.ranges[i];
      intensity_scan.intensities[i] = laser_msg->laser_scan.intensities[i];
     }
    else{
      intensity_scan.ranges[i] = std::numeric_limits<float>::infinity();
      intensity_scan.intensities[i] = 0;
    }

  intensity_scan.header.stamp = ros::Time::now();

  intensity_pub.publish(intensity_scan);
}

void rawDataCallback(const sick_safetyscanners::RawMicroScanDataMsgConstPtr& raw_data){
  std_msgs::Int8MultiArray zone_intrusion;
  zone_intrusion.data.resize(2); //Position [0] - protective zone, [1] - warning zone
  zone_intrusion.data[0] = 0;
  zone_intrusion.data[1] = 0;

  int array_size = raw_data->measurement_data.number_of_beams;
  //Protective Zone
  for (int i=0; i<array_size; i++)
    if(raw_data->intrusion_data.data[0].flags[i]){
      zone_intrusion.data[0] = 1;
      break;
    }

  //Warning Zone
  for (int i=0; i<array_size; i++)
    if(raw_data->intrusion_data.data[1].flags[i]){
      zone_intrusion.data[1] = 1;
      break;
    }

  protective_fields.publish(zone_intrusion);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "IntesityScanNode");
  ros::NodeHandle nh;

  loadParams();

  intensity_pub = nh.advertise<sensor_msgs::LaserScan>(publish_topic, 100);
  protective_fields = nh.advertise<std_msgs::Int8MultiArray>("protective_fileds", 100);

  ros::Subscriber laser_sub = nh.subscribe<sick_safetyscanners::ExtendedLaserScanMsg>(input_topic, 1, laserScanCallback);
  ros::Subscriber raw_msg = nh.subscribe<sick_safetyscanners::RawMicroScanDataMsg>(raw_topic, 1, rawDataCallback);

  ros::spin();

  return 0;
}
