#include <ros/package.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x, y, theta;
};

std::vector<Goal> loadGoals(const std::string& filename) {
    std::vector<Goal> goals;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Goal g;
        char comma;
        ss >> g.x >> comma >> g.y >> comma >> g.theta;
        goals.push_back(g);
    }
    return goals;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base action server.");

    std::string filepath = ros::package::getPath("second-robotics-project-polimi") + "/csv/goals.csv";
    std::vector<Goal> goals = loadGoals(filepath);

    for (size_t i = 0; i < goals.size(); ++i) {
        const Goal& g = goals[i];
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = g.x;
        goal.target_pose.pose.position.y = g.y;
        goal.target_pose.pose.orientation.z = sin(g.theta / 2.0);
        goal.target_pose.pose.orientation.w = cos(g.theta / 2.0);

        ROS_INFO("Sending goal %lu: [x=%.2f y=%.2f theta=%.2f]", i, g.x, g.y, g.theta);
        ac.sendGoal(goal);

        bool finished = ac.waitForResult(ros::Duration(60.0));
        if (finished && ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %lu reached!", i);
        } else {
            ROS_WARN("Goal %lu failed or was aborted. Continuing...", i);
        }

        ros::Duration(2.0).sleep();  // small pause between goals
    }

    ROS_INFO("All goals sent.");
    return 0;
}
