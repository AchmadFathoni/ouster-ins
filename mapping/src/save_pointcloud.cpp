/*
 * Save last message of PointCloud2 from a topic.
 * Param:
 *  topic : Topic of the message that will be saved.
 *  path : Complete file path of saved .pcd file. Default is saved.pcl in current working directory
 */

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

using sensor_msgs::msg::PointCloud2;
using std_srvs::srv::Empty;
using std::placeholders::_1, std::placeholders::_2;
using Pointcloud = pcl::PointCloud<pcl::PointXYZ>;
using std::string, std::map;

const string node_name = "save_pointcloud";
const string default_path = (std::filesystem::current_path() / "saved.pcd").generic_string();

class SavePointCloud : public rclcpp::Node {
public:
    SavePointCloud() : Node(node_name) {
        this->declare_parameters(node_name, params_);
        this->get_parameters(node_name, params_);

        sub_ = this->create_subscription<PointCloud2>(
            params_["topic"], 1, std::bind(&SavePointCloud::sub_cb_, this, _1)
        );
        srv_ = this->create_service<Empty>(
            "save_map", std::bind(&SavePointCloud::srv_cb_, this, _1, _2)
        );
    }
private:
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
    rclcpp::Service<Empty>::SharedPtr srv_;
    PointCloud2::SharedPtr current_cloud_;

    map<string, string> params_{{"topic", "/filtered"}, {"path", default_path}};

    void sub_cb_(const PointCloud2::SharedPtr in_cloud) {
        current_cloud_ = in_cloud;
    }

    void srv_cb_([[maybe_unused]]const std::shared_ptr<Empty::Request> req,
                 [[maybe_unused]]const std::shared_ptr<Empty::Response> res)
    const {
        Pointcloud pcl_cloud;
        pcl::fromROSMsg(*current_cloud_, pcl_cloud);
        pcl::io::savePCDFileBinaryCompressed(params_.at("path"), pcl_cloud);
        RCLCPP_INFO_ONCE(this->get_logger(), "Pointcloud is saved.");
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SavePointCloud>());
    return 0;
}
