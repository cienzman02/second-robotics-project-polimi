#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class ScanMerger {
public:
    ScanMerger() {
        front_sub = nh.subscribe("/scan_front", 1, &ScanMerger::frontCallback, this);
        back_sub = nh.subscribe("/scan_back", 1, &ScanMerger::backCallback, this);
        scan_pub = nh.advertise<sensor_msgs::LaserScan>("/merged_scan", 1);
    }

    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        front_scan = *msg;
        tryMerge();
    }

    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        back_scan = *msg;
        tryMerge();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber front_sub, back_sub;
    ros::Publisher scan_pub;
    sensor_msgs::LaserScan front_scan, back_scan;
    bool front_ready = false, back_ready = false;

    void tryMerge() {
        // Wait until we receive valid scans from both sensors before merging
        if (!front_ready && !front_scan.ranges.empty()) front_ready = true;
        if (!back_ready && !back_scan.ranges.empty()) back_ready = true;

        if (!front_ready || !back_ready) return;

        // Create merged scan
        sensor_msgs::LaserScan merged = front_scan;
        merged.angle_min = -M_PI;
        merged.angle_max = M_PI;
        merged.angle_increment = front_scan.angle_increment;
        int total_size = front_scan.ranges.size() + back_scan.ranges.size();
        merged.ranges.resize(total_size);
        merged.intensities.resize(total_size);

        // Merge front
        for (size_t i = 0; i < front_scan.ranges.size(); ++i) {
            float r = front_scan.ranges[i];
            merged.ranges[i] = (r > 0.35 && r < front_scan.range_max) ? r : std::numeric_limits<float>::infinity();
        }
        
        // Merge back (append after front)
        for (size_t i = 0; i < back_scan.ranges.size(); ++i) {
            float r = back_scan.ranges[i];
            int idx = i + front_scan.ranges.size();
            merged.ranges[idx] = (r > 0.35 && r < back_scan.range_max) ? r : std::numeric_limits<float>::infinity();
        }

        merged.header.stamp = ros::Time::now();
        merged.header.frame_id = "base_link";

        scan_pub.publish(merged);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_merger");
    ScanMerger sm;
    ros::spin();
    return 0;
}
