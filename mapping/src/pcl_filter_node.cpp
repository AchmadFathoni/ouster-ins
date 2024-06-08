#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using Pointcloud = pcl::PointCloud<pcl::PointXYZ>;
using std::pow, std::string, std::map;

const string node_name = "pcl_filter";

class Filter : public rclcpp::Node {
public:
    Filter() : Node(node_name) {
        this->declare_parameters(node_name, params_);
        this->get_parameters(node_name, params_);
        this->declare_parameter(node_name + ".min_depth", 0.9);
        min_depth_square_ = pow(this->get_parameter(node_name + ".min_depth").as_double(), 2);

        sub_ = this->create_subscription<PointCloud2>(params_["in_topic"], 1, std::bind(&Filter::callback, this, _1));
        pub_ = this->create_publisher<PointCloud2>(params_["out_topic"], 1);
    }
private:
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
    map<string, string> params_{{"in_topic", "/ouster/points"}, {"out_topic", "/filtered"}};
    double min_depth_square_;
    void callback(const PointCloud2::SharedPtr in_cloud) const {
        Pointcloud pcl_cloud;
        pcl::fromROSMsg(*in_cloud, pcl_cloud);

        auto& points = pcl_cloud.points;
        for (std::size_t p = 0; p < pcl_cloud.points.size(); ++p) {
            auto& point = points[p];
            auto depth_square = pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2);
            if (depth_square < min_depth_square_) {
                point = points[points.size() - 1];
                points.resize(points.size() - 1);
                --p;
            }
        }

        PointCloud2 out_cloud;
        pcl::toROSMsg(pcl_cloud, out_cloud);
        out_cloud.header = in_cloud->header;
        out_cloud.height = 1;
        out_cloud.width = points.size();
        pub_->publish(out_cloud);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Filter>());
    rclcpp::shutdown();
    return 0;
}
