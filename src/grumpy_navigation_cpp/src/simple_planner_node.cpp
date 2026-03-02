
#include "rclcpp/rclcpp.hpp"
#include "geometry_msg/msg/pose_stamped.hpp"
#include "geometry_msg/msg/transformed_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SimplePlannerNode : public rclcpp::Node {
public:
    SimplePlannerNode() : Node("path_planner_node"){
        declare_parameter<std::string>("world_frame", "map");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<int>("num_points", 40);

        world
    }
};
