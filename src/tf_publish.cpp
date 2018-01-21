#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

class PublisherImu
{
private:

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber imu_sub_;
  std::string frame_id_;
  geometry_msgs::Vector3 translation_;


  void imuCallback(const sensor_msgs::ImuConstPtr &msg)
  {
    geometry_msgs::TransformStamped imu_tf;
    imu_tf.header.stamp = ros::Time::now();
    imu_tf.header.frame_id = "base_link";
    imu_tf.child_frame_id = frame_id_;
    imu_tf.transform.translation = translation_;
    imu_tf.transform.rotation = msg->orientation;
    tf_broadcaster_.sendTransform(imu_tf);
  }

public:

  PublisherImu(ros::NodeHandle& nh_priv, std::string topic, std::string frame_id, double x)
  {
    frame_id_ = frame_id;
    translation_.x = x;
    translation_.y = 0;
    translation_.z = 0;
    imu_sub_ = nh_priv.subscribe<sensor_msgs::Imu>(topic, 10, &PublisherImu::imuCallback, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_publish");
  ros::NodeHandle nh{""};
  ros::NodeHandle nh_priv{"~"};

#if 0
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("imu/calibrate");

  ROS_INFO("Waiting for calibration service");
  client.waitForExistence();
  ROS_INFO("Service is up!");

  std_srvs::Empty srv;
  if (client.call(srv))
  {
    ROS_INFO("Gyro calibration succeeded");
  }
  else
  {
    ROS_ERROR("Gyro calibration failed");
    return 1;
  }
#endif

  PublisherImu publisher_stateless{nh_priv, "/imu/data_stateless", "imu_link_stateless", 0.0};
  PublisherImu publisher_madgwick{nh_priv, "/imu/data_madgwick", "imu_link_madgwick", -0.1};
  PublisherImu publisher_complementary{nh_priv, "/imu/data_complementary", "imu_link_complementary", 0.1};

  ros::spin();
  return 0;
}
