#include <cstring>
#include <memory>
#include <utility>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class LivoxCloudConverter : public rclcpp::Node
{
public:
  LivoxCloudConverter() : Node("livox_cloud_converter")
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_pc2", 10);
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", 10,
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
        convert(*message);
      });
  }

private:
  static sensor_msgs::msg::PointField field(
    const char * name, uint32_t offset, uint8_t datatype)
  {
    sensor_msgs::msg::PointField result;
    result.name = name;
    result.offset = offset;
    result.datatype = datatype;
    result.count = 1;
    return result;
  }

  void convert(const livox_ros_driver2::msg::CustomMsg & message)
  {
    using Field = sensor_msgs::msg::PointField;
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header = message.header;
    cloud->height = 1;
    cloud->width = message.points.size();
    cloud->fields = {
      field("x", 0, Field::FLOAT32),
      field("y", 4, Field::FLOAT32),
      field("z", 8, Field::FLOAT32),
      field("intensity", 12, Field::FLOAT32),
      field("tag", 16, Field::UINT8),
      field("line", 17, Field::UINT8),
      field("timestamp", 20, Field::FLOAT64),
    };
    const uint16_t byte_order = 1;
    cloud->is_bigendian = *reinterpret_cast<const uint8_t *>(&byte_order) == 0;
    cloud->point_step = 28;
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->is_dense = true;
    cloud->data.resize(cloud->row_step);
    for (size_t index = 0; index < message.points.size(); ++index) {
      const auto & point = message.points[index];
      auto * output = cloud->data.data() + index * cloud->point_step;
      const float intensity = point.reflectivity;
      // Match the nanosecond timestamp used by the existing offline converter.
      const double timestamp = static_cast<double>(message.timebase + point.offset_time);
      std::memcpy(output, &point.x, sizeof(point.x));
      std::memcpy(output + 4, &point.y, sizeof(point.y));
      std::memcpy(output + 8, &point.z, sizeof(point.z));
      std::memcpy(output + 12, &intensity, sizeof(intensity));
      output[16] = point.tag;
      output[17] = point.line;
      std::memcpy(output + 20, &timestamp, sizeof(timestamp));
    }
    publisher_->publish(std::move(cloud));
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
