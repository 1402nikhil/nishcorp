#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cmath>
#include <array>
#include <algorithm>

#include "example_interfaces/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std;

class CalculateJointAnglesNode : public rclcpp::Node
{

    struct Vec3 {
        double x, y, z;
    };

public:
    CalculateJointAnglesNode() : Node("calculate_joint_angles")
    {
        num_pos_subs_ = this->create_subscription<example_interfaces::msg::Int32>(
            "pos_num", 10,
            std::bind(&CalculateJointAnglesNode::callbackPosNum, this, std::placeholders::_1));

        pos_subs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "bot_position", 10,
            std::bind(&CalculateJointAnglesNode::callbackBotPosition, this, std::placeholders::_1));

        arm_joints_cont_1_subs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "arm_joints_controller_1/commands", 10);

        arm_joints_cont_2_subs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "arm_joints_controller_2/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Calculate Joint Angle started");
    }

    array<array<double, 3>, 3> roty(double theta) {
        return {{
            {cos(theta), -sin(theta), 0},
            {sin(theta),  cos(theta), 0},
            {0,           0,          1}
        }};
    }

    void transform(
        array<Vec3, 4> corners,
        array<Vec3, 2> edges,
        double x_trans, double y_trans, double theta_y,
        array<Vec3, 4>& new_corners,
        array<Vec3, 2>& new_edges
    ) {
        auto R = roty(theta_y);
        Vec3 T = {x_trans, y_trans, 0};

        for (int i = 0; i < 4; ++i) {
            new_corners[i].x = R[0][0]*corners[i].x + R[0][1]*corners[i].y + R[0][2]*corners[i].z + T.x;
            new_corners[i].y = R[1][0]*corners[i].x + R[1][1]*corners[i].y + R[1][2]*corners[i].z + T.y;
            new_corners[i].z = R[2][0]*corners[i].x + R[2][1]*corners[i].y + R[2][2]*corners[i].z + T.z;
        }

        for (int i = 0; i < 2; ++i) {
            new_edges[i].x = R[0][0]*edges[i].x + R[0][1]*edges[i].y + R[0][2]*edges[i].z + T.x;
            new_edges[i].y = R[1][0]*edges[i].x + R[1][1]*edges[i].y + R[1][2]*edges[i].z + T.y;
            new_edges[i].z = R[2][0]*edges[i].x + R[2][1]*edges[i].y + R[2][2]*edges[i].z + T.z;
        }
    }

    // -----------------------
    // Inverse Kinematics (2-DOF leg)
    // -----------------------
    pair<double, double> inverse_leg_xy(Vec3 foot_pos, Vec3 base_pos, double a1, double a2) {
        double dx = foot_pos.x - base_pos.x;
        double dy = foot_pos.y - base_pos.y;
        double D = (dx*dx + dy*dy - a1*a1 - a2*a2) / (2 * a1 * a2);
        D = max(-1.0, min(1.0, D)); // numerical safety

        double theta2 = atan2(sqrt(1 - D*D), D); // elbow-down
        double theta1 = atan2(dy, dx) - atan2(a2*sin(theta2), a1 + a2*cos(theta2));
        return {theta1, theta2};
    }

    void calculateAngles(double coordinates[3]){

        x_trans0 = coordinates[0] * M_PI / 180.0 ;
        y_trans0 = coordinates[1] * M_PI / 180.0 ;
        theta_y0 = coordinates[2] * M_PI / 180.0 ;

        transform(corners, edge_points, x_trans0, y_trans0, theta_y0, new_corners, new_edges);

        // Fixed foot positions (on ground)
        Vec3 foot1_fixed = new_edges[0];
        // Vec3 foot2_fixed = new_edges[1];
        foot1_fixed.y = -2.0;
        // foot2_fixed.y = -2.0;

        // Inverse Kinematics for both legs
        auto [theta1_leg1, theta2_leg1] = inverse_leg_xy(foot1_fixed, new_edges[0], a1, a2);
        // auto [theta1_leg2, theta2_leg2] = inverse_leg_xy(foot2_fixed, new_edges[1], a1, a2);

        auto data = std_msgs::msg::Float64MultiArray();

        data.data = {theta1_leg1, theta2_leg1};

        arm_joints_cont_1_subs_ ->publish(data);
        arm_joints_cont_2_subs_ ->publish(data);

        RCLCPP_INFO(this->get_logger(), "BotPosition: Angles: %f, %f", data.data[0], data.data[1]);
    }

private:

    void callbackBotPosition(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        double coordinates[3] = {msg->data[0], msg->data[1], msg->data[2]};
        calculateAngles(coordinates);
    }

    void callbackPosNum(const example_interfaces::msg::Int32::SharedPtr msg)
    {
        float poss[2] = {0, 0};

        switch (msg->data) {
            case 1:
                poss[0] = pos[0][0];
                poss[1] = pos[0][1];
                break;
            case 2:
                poss[0] = pos[1][0];
                poss[1] = pos[1][1];
                break;
            case 3:
                poss[0] = pos[2][0];
                poss[1] = pos[2][1];
                break;
            case 4:
                poss[0] = pos[3][0];
                poss[1] = pos[3][1];
                break;
            case 5:
                poss[0] = pos[4][0];
                poss[1] = pos[4][1];
                break;
            default:
                break; // Optional: last break is not strictly necessary but good practice
        }

        poss[0] = poss[0] * M_PI / 180.0 ;
        poss[1] = poss[1] * M_PI / 180.0 ;

        auto data = std_msgs::msg::Float64MultiArray();

        data.data = {poss[0], poss[1]};

        arm_joints_cont_1_subs_ ->publish(data);
        arm_joints_cont_2_subs_ ->publish(data);

        RCLCPP_INFO(this->get_logger(), "PosNum: Angles: %f, %f", data.data[0], data.data[1]);

    }

    float pos[5][2] =  {{0.0, 0.0}, 
                        {80.0, 70.0},
                        {105.0, 130.0},
                        {40.0, 0.0},
                        {90.0, 0.0},};

    const double a1 = 1.5;
    const double a2 = 1.5;
    const double a3 = 1.5;


    // Body dimensions
    double length = 4.0, width = 2.0;

    array<Vec3, 4> corners = {{
        {-length/2, 0, -width/2},
        { length/2, 0, -width/2},
        { length/2, 0,  width/2},
        {-length/2, 0,  width/2}
    }};

    array<Vec3, 2> edge_points = {{
        {1.0, 0, -width/2},  // Right leg mount
        {1.0, 0,  width/2}   // Left leg mount
    }};

    // Transform parameters
    double x_trans0 = 0.0, y_trans0 = 0.0, theta_y0 = 0.0;

    array<Vec3, 4> new_corners;
    array<Vec3, 2> new_edges;

    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr num_pos_subs_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_subs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_joints_cont_1_subs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_joints_cont_2_subs_;
};

int main(int argc, char **argv)
{ 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalculateJointAnglesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
