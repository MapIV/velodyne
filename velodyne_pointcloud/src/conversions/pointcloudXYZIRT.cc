


#include <velodyne_pointcloud/pointcloudXYZIRT.h>

namespace velodyne_pointcloud 
{

  PointcloudXYZIRT::PointcloudXYZIRT(
    const double max_range, const double min_range,
    const std::string& target_frame, const std::string& fixed_frame,
    const unsigned int scans_per_block, boost::shared_ptr<tf::TransformListener> tf_ptr)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        0, 1, true, scans_per_block, tf_ptr, 7,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "stamp_sec", 1, sensor_msgs::PointField::UINT32,
        "stamp_nsec", 1, sensor_msgs::PointField::UINT32),
        iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"),
        iter_ring(cloud, "ring"), iter_intensity(cloud, "intensity"),
        iter_stamp_sec(cloud, "stamp_sec"), iter_stamp_nsec(cloud, "stamp_nsec")
    {};

  void PointcloudXYZIRT::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
    iter_stamp_sec = sensor_msgs::PointCloud2Iterator<uint32_t >(cloud, "stamp_sec");
    iter_stamp_nsec = sensor_msgs::PointCloud2Iterator<uint32_t >(cloud, "stamp_nsec");
  }

  void PointcloudXYZIRT::newLine()
  {}

  void PointcloudXYZIRT::addPoint(float x, float y, float z, uint16_t ring, uint16_t /*azimuth*/, float distance, float intensity)
  {}

  void PointcloudXYZIRT::addPoint_T(float x, float y, float z, uint16_t ring, uint16_t /*azimuth*/, float distance, float intensity,
                                  uint32_t stamp_sec, uint32_t stamp_nsec)
  {
    if(!pointInRange(distance)) return;

    // convert polar coordinates to Euclidean XYZ

    if(config_.transform)
      transformPoint(x, y, z);

    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    *iter_ring = ring;
    *iter_intensity = intensity;
    *iter_stamp_sec = stamp_sec;
    *iter_stamp_nsec = stamp_nsec;

    ++cloud.width;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_ring;
    ++iter_intensity;
    ++iter_stamp_sec;
    ++iter_stamp_nsec;
  }

}

