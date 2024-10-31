#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // Correct header for LaserScan
#include "cmath"
#include "nav_msgs/msg/map_meta_data.hpp"  // Correct header for MapMetaData
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"       // Correct header for Pose

using namespace std::chrono_literals;
using std::placeholders::_1;

//CONSTANTS
#define MIN_DISTANCE 0.12  // Min distance (meters)
#define MAX_DISTANCE 10 // Max distance (meters)
#define DIMENSIONS 200  // The grid dimensions
#define RESOLUTION 0.1  // Resolution of map (meters)
#define LIDAR_POS 100   // Row and Col indices for LiDAR


class lidar_map_node : public rclcpp::Node
{
  public:
    lidar_map_node() : Node("lidar_map_node")
    {
      map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/lidar_map", 10);
      scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&lidar_map_node::onLidarCallback, this, _1));
    }
    

  private:
    void onLidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	//Instantiate message to publish to
	nav_msgs::msg::OccupancyGrid map_data;

	// Change the map meta data. Can also choose origin?
	nav_msgs::msg::MapMetaData map_info;
	map_info.resolution = RESOLUTION;
	map_info.width = DIMENSIONS;
	map_info.height = DIMENSIONS;

	geometry_msgs::msg::Pose origin;
	origin.position.x = -DIMENSIONS / 2 * RESOLUTION;
	origin.position.y = -DIMENSIONS / 2 * RESOLUTION;
	map_info.origin = origin;

	// Give map meta data to occupancy grid
	map_data.info = map_info;
	// Set up a vector to handle the lidar data
	std::vector<int8_t> lidar_data(DIMENSIONS*DIMENSIONS);
	//Instantiate variables for obstacle detection
	float angle;
	std::vector<float> ranges = msg->ranges;

	for (long unsigned int i = 0; i < ranges.size(); i++)
	{
		// If something is detected within range
		if (ranges[i] > MIN_DISTANCE && ranges[i] <= MAX_DISTANCE)
		{
			// Calculate angle
			angle = msg->angle_min + (i * msg->angle_increment);
			// Get cartesian coordinates
			float x = ranges[i] * cos(angle);
			float y = ranges[i] * sin(angle);
			// Convert to number of indices away from LiDAR
			int index = (LIDAR_POS - round(y / RESOLUTION)) * DIMENSIONS + (LIDAR_POS - round(x / RESOLUTION));
			// Save value to save on text
			int cur_val = lidar_data[index];
			// If the indice already has an obstacle, increment "certainty"
			if (cur_val > 0 && cur_val < 100)
			{
				lidar_data[index] += 1;
			}
			else if (cur_val != 100)
			{
				lidar_data[index] = 1;
			}
			//lidar_data[index] = 100;
		}
	}
	// Set anything in the occupancy grid without a value to 0
	for (long int i = 0; i < DIMENSIONS * DIMENSIONS; ++i)
	{
		if (lidar_data[i] > 0 && lidar_data[i] <= 100)
			lidar_data[i] = 100;
		if (lidar_data[i] != 100)
			lidar_data[i] = 0;
	}

	map_data.data = lidar_data;
	map_pub->publish(map_data);
	std::stringstream ss;
    for (size_t i = 0; i < map_data.data.size(); ++i) {
        ss << static_cast<int>(map_data.data[i]) << " ";
    }

    RCLCPP_INFO(this->get_logger(), "LIDAR Data: '%s'", ss.str().c_str());


}   
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;


};
/**
 * @brief
 *
 * @param argc
 * @param argv  
 * @return int
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_map_node>());
  rclcpp::shutdown();
  return 0;
}
