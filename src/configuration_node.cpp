#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vision_challenge/config.h>
#include <vision_challenge/solvePosition.h>

using namespace Eigen;

// Validate Solution by re-projecting solution to pixel coordinates
bool validateSolution(const vision_challenge::solvePosition::Request &req,
                      vision_challenge::solvePosition::Response &res,
                      const Eigen::Matrix<double, 3, 4> &P1, const Eigen::Matrix<double, 3, 4> &P2,
                      const Eigen::Vector2d &d1, const Eigen::Vector2d &d2)
{
    Eigen::Vector4d solution(req.object_position.x, req.object_position.y, req.object_position.z, 1);

    // Reproject the solution to camera 1
    Eigen::Vector3d reproj_d1 = P1 * solution;
    reproj_d1 /= reproj_d1.z();

    // Reproject the solution to camera 2
    Eigen::Vector3d reproj_d2 = P2 * solution;
    reproj_d2 /= reproj_d2.z();

    // Compare the reprojected coordinates with the original detections
    double tolerance = 1; // Tolerance in pixels
    if ((reproj_d1.head<2>() - d1).norm() < tolerance && (reproj_d2.head<2>() - d2).norm() < tolerance)
    {
        res.success = true;
        res.message = "Solution validated successfully.";
        return true;
    }
    else
    {
        res.success = false;
        res.message = "Solution validation failed.";
        return false;
    }
}

// Callback function to connect to solution node and perform validation
bool solutionCallback(vision_challenge::solvePosition::Request &req,
                      vision_challenge::solvePosition::Response &res,
                      const Eigen::Matrix3d &K,
                      const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                      const Eigen::Vector2d &d1, const Eigen::Vector2d &d2,
                      double theta1, double theta2)
{
    tf::TransformBroadcaster tf_broadcaster;

    // Calculate camera's extrinsic matrices for each position
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

    // Perform validation
    bool isValid = validateSolution(req, res, P1, P2, d1, d2);

    // If solution is valid, display tf frame of object
    if (isValid)
    {
        tf::Transform transform_turtle;
        transform_turtle.setOrigin(tf::Vector3(req.object_position.x, req.object_position.y, req.object_position.z));
        tf::Quaternion q_turtle;
        q_turtle.setRPY(0, 0, 0);
        transform_turtle.setRotation(q_turtle);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform_turtle, ros::Time::now(), "world", "bird"));
    }
    else
    {
        ROS_WARN_STREAM("Solution validation failed: " << res.message);
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "configuration_node");
    ros::NodeHandle nh;
    ros::Publisher config_pub = nh.advertise<vision_challenge::config>("configuration", 1);
    tf::TransformBroadcaster tf_broadcaster;

    int camera_height, camera_width;
    double camera_focal_length;

    // Load camera parameters from the YAML file
    ros::param::get("cam_info/height", camera_height);
    ros::param::get("cam_info/width", camera_width);
    ros::param::get("cam_info/focal_length", camera_focal_length);

    // Create the intrinsic camera matrix
    Matrix3d K;
    K << camera_focal_length, 0, camera_width / 2,
         0, camera_focal_length, camera_height / 2,
         0, 0, 1;

    // Set configuration parameters
    vision_challenge::config config_msg;
    config_msg.p1.x = 1.0;
    config_msg.p1.y = 2.0;
    config_msg.p1.z = 1.0;
    config_msg.theta1 = -20.0;
    config_msg.d1.x = 380.0;
    config_msg.d1.y = 290.0;

    config_msg.p2.x = 6.5;
    config_msg.p2.y = 3.0;
    config_msg.p2.z = 2.0;
    config_msg.theta2 = 30.0;
    config_msg.d2.x = 138.0;
    config_msg.d2.y = 367.0;

    Eigen::Vector3d p1(config_msg.p1.x, config_msg.p1.y, config_msg.p1.z);
    Eigen::Vector3d p2(config_msg.p2.x, config_msg.p2.y, config_msg.p2.z);
    Eigen::Vector2d d1(config_msg.d1.x, config_msg.d1.y);
    Eigen::Vector2d d2(config_msg.d2.x, config_msg.d2.y);
    double theta1 = config_msg.theta1 * M_PI / 180.0;
    double theta2 = config_msg.theta2 * M_PI / 180.0;

    // Validate the position received from solution node
    ros::ServiceServer validation_srv = nh.advertiseService<vision_challenge::solvePosition::Request, vision_challenge::solvePosition::Response>(
        "solve_position",
        boost::bind(solutionCallback, _1, _2, K, p1, p2, d1, d2, -theta1, -theta2));

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // Create and broadcast the p1 frame
        tf::Transform transform_p1;
        transform_p1.setOrigin(tf::Vector3(p1.x(), p1.y(), p1.z()));
        tf::Quaternion q_p1;
        q_p1.setRPY(0, 0, theta1);
        transform_p1.setRotation(q_p1);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform_p1, ros::Time::now(), "world", "robot_cam1"));

        // Create and broadcast the p2 frame
        tf::Transform transform_p2;
        transform_p2.setOrigin(tf::Vector3(p2.x(), p2.y(), p2.z()));
        tf::Quaternion q_p2;
        q_p2.setRPY(0, 0, theta2);
        transform_p2.setRotation(q_p2);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform_p2, ros::Time::now(), "world", "robot_cam2"));

        config_pub.publish(config_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}