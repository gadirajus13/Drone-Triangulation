#include <ros/ros.h>
#include <Eigen/Dense>
#include <vision_challenge/config.h>
#include <vision_challenge/solvePosition.h>
#include <boost/function.hpp>
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solver_node");
    ros::NodeHandle nh;

    Eigen::Vector3d p1, p2;
    Eigen::Vector2d d1, d2;
    double theta1, theta2;
    double focal_length;
    int camera_width, camera_height;

    // Load camera parameters from the YAML file
    ros::param::get("cam_info/width", camera_width);
    ros::param::get("cam_info/height", camera_height);
    ros::param::get("cam_info/focal_length", focal_length);

    boost::function<void(const vision_challenge::config::ConstPtr&)> callback =
        [&](const vision_challenge::config::ConstPtr &msg) {
            p1 << msg->p1.x, msg->p1.y, msg->p1.z;
            p2 << msg->p2.x, msg->p2.y, msg->p2.z;
            d1 << msg->d1.x, msg->d1.y;
            d2 << msg->d2.x, msg->d2.y;
            theta1 = -msg->theta1 * M_PI / 180.0;
            theta2 = -msg->theta2 * M_PI / 180.0;
            
            // Intrinsic Camera Matrix
            Matrix3d K;
            K << focal_length, 0, camera_width/2,
                 0, focal_length, camera_height/2,
                 0, 0, 1;

            Matrix3d R_x;
            R_x << 1,  0,  0,
                0,  0, -1,
                0,  1,  0;

            Eigen::Matrix3d R1_z;
            R1_z << cos(theta1), -sin(theta1), 0,
                sin(theta1),  cos(theta1), 0,
                0, 0, 1;

            Eigen::Matrix3d R2_z;
            R2_z << cos(theta2), -sin(theta2), 0,
                sin(theta2),  cos(theta2), 0,
                0, 0, 1;

            // Create translation vectors
            Eigen::Vector3d t1= p1;
            Eigen::Vector3d t2= p2;

            // Create extrinsic matrices
            Matrix3d R1 = R_x * R1_z;
            Eigen::Matrix4d T1;
            T1.block<3, 3>(0, 0) = R1;
            T1.block<3, 1>(0, 3) = -R1 * t1;
            T1.row(3) << 0, 0, 0, 1;

            Matrix3d R2 = R_x * R2_z;
            Eigen::Matrix4d T2;
            T2.block<3, 3>(0, 0) = R2;
            T2.block<3, 1>(0, 3) = -R2 * t2;
            T2.row(3) << 0, 0, 0, 1;

            Eigen::Matrix<double, 3, 4> P1 = K * T1.block<3, 4>(0, 0);
            Eigen::Matrix<double, 3, 4> P2 = K * T2.block<3, 4>(0, 0);

            // Solve for the intersection point
            Matrix<double, 4, 4> A;
            A.row(0) = d1.x() * P1.row(2) - P1.row(0);
            A.row(1) = d1.y() * P1.row(2) - P1.row(1);
            A.row(2) = d2.x() * P2.row(2) - P2.row(0);
            A.row(3) = d2.y() * P2.row(2) - P2.row(1);

            Vector4d b = Vector4d::Zero();
            b(3) = 1;

            // QR Decomposition to solve linear system
            Vector4d t = A.fullPivHouseholderQr().solve(b);

            // Normalize the vector by scaling factor
            Vector3d object_pos = t.head<3>() / t(3);

            // Send solve position request
            ros::ServiceClient solve_position_client = nh.serviceClient<vision_challenge::solvePosition>("solve_position");
            vision_challenge::solvePosition srv;
            srv.request.object_position.x = object_pos.x();
            srv.request.object_position.y = object_pos.y();
            srv.request.object_position.z = object_pos.z();

            if (solve_position_client.call(srv))
            {
                if (srv.response.success)
                {
                    ROS_INFO("Solution found: (%.2f, %.2f, %.2f) ",object_pos.x(),object_pos.y(),object_pos.z());
                }
                else
                {
                    ROS_WARN_STREAM("Solution validation failed: " << srv.response.message);
                }
            }
            else
            {
                ROS_ERROR("Failed to call solve position service!");
            }
            
        };

    ros::Subscriber config_sub = nh.subscribe("configuration", 1, callback);

    ros::spin();
    return 0;
}