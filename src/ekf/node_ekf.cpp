#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <math.h>
#include <deque>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <include/euler_q_rmatrix.h>
#include <ekf/ekf_model.h>

using namespace std;
using namespace Eigen;

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_world;
extern rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_array;
extern Mat12x12 Q;
extern Mat6x6 R_vo;
extern deque<MatrixXd> state_q;
extern deque<MatrixXd> covariance_q;
extern deque<sensor_msgs::msg::Imu> imu_msg_q;

extern rclcpp::Time last_frame_time;
extern rclcpp::Time last_imu_time;
extern int initialized;
extern int got_first_imu;

extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vel;
extern Eigen::Quaterniond q_gt;
extern Eigen::Vector3d position_gt, velocity_gt;


rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_world;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_array;
Mat12x12 Q;
Mat6x6 R_vo;
deque<MatrixXd> state_q;
deque<MatrixXd> covariance_q;
deque<sensor_msgs::msg::Imu> imu_msg_q;

rclcpp::Time last_frame_time;
rclcpp::Time last_imu_time;
int initialized = 0;
int got_first_imu = 0;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vel;
Eigen::Quaterniond q_gt;
Eigen::Vector3d position_gt, velocity_gt;


void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    //your code for propagation
    sensor_msgs::msg::Imu drift_msg;
    drift_msg = *msg;
    rclcpp::Duration driftT(0,0);
    driftT = rclcpp::Duration::from_seconds(0.0);
    rclcpp::Time msg_time = msg->header.stamp;
    msg_time=msg_time-driftT;
    drift_msg.header.stamp=msg_time;
    //    cout << "orig msg time " <<msg->header.stamp << endl;
    //    cout << "move with the drift" << msg_time << endl;
    if (!initialized) // if initialized start computing
    {
        //cout << "waiting for init" << endl;
        last_imu_time = msg_time;
        return;
    }
    if (!got_first_imu)
    {
        cout << "inited and got imu data" << endl
             << "at time :" << msg_time.seconds() << endl
             << endl
             << endl;
        imu_msg_q.push_back(*msg);
        last_imu_time = msg_time;
        got_first_imu = 1;
        return;
    }

    Vec15 x_prev;
    Mat15x15 cov_prev;
    Vec15 x_new;
    Mat15x15 cov_new;

    //recall last state from q
    x_prev = state_q.back();
    cov_prev = covariance_q.back();

    //input
    Vec6 u;
    Vec12 n;
    n.setZero(12);
    u[0] = msg->angular_velocity.x;
    u[1] = msg->angular_velocity.x;
    u[2] = msg->angular_velocity.x;
    u[3] = msg->linear_acceleration.x;
    u[4] = msg->linear_acceleration.y;
    u[5] = msg->linear_acceleration.z;

    //predict new state
    Vec15 x_dot;
    x_dot = xdot(x_prev, u, n);
    double dT = (msg_time - last_imu_time).seconds();
    x_new = x_prev + (dT * x_dot);

    //Calculate F V
    Mat15x15 A = jacobian_A(x_prev, u, n);
    Mat15x12 U = jacobian_U(x_prev, u, n);
    Mat15x15 I15x15;
    I15x15.setIdentity();
    Mat15x15 F = I15x15 + dT * A;
    Mat15x12 V = dT * U;

    //Calculate cov matrix
    cov_new = F * cov_prev * F.transpose() + V * Q * V.transpose();

    //cout << cov_new << endl;
    last_imu_time = msg_time;
    imu_msg_q.push_back(drift_msg);
    state_q.push_back(x_new);
    covariance_q.push_back(cov_new);
    if (imu_msg_q.size() > 500)
    {
        imu_msg_q.pop_front();
        state_q.pop_front();
        covariance_q.pop_front();
    }

    //publish nav_msgs::msg::Odometry
    nav_msgs::msg::Odometry ekf_odom;
    ekf_odom.header.stamp = msg_time;
    ekf_odom.header.frame_id = "world";
    ekf_odom.pose.pose.position.x = x_new(0);
    ekf_odom.pose.pose.position.y = x_new(1);
    ekf_odom.pose.pose.position.z = x_new(2);
    ekf_odom.twist.twist.linear.x = x_new(6);
    ekf_odom.twist.twist.linear.y = x_new(7);
    ekf_odom.twist.twist.linear.z = x_new(8);
    Quaterniond odom_q;
    odom_q = rotation_matrix_from_euler(Vector3d(x_new(3), x_new(4), x_new(5)));
    odom_q.normalize();
    ekf_odom.pose.pose.orientation.w = odom_q.w();
    ekf_odom.pose.pose.orientation.x = odom_q.x();
    ekf_odom.pose.pose.orientation.y = odom_q.y();
    ekf_odom.pose.pose.orientation.z = odom_q.z();
    odom_pub->publish(ekf_odom);
}

//error state injection
void repredict(MatrixXd mil_camera, MatrixXd covariance_camera, rclcpp::Time last_frame_time, unsigned int index)
{
    Vec15 x_new;
    Mat15x15 cov_new;
    Vec15 x_prev;
    Mat15x15 cov_prev;

    x_prev = mil_camera;
    cov_prev = covariance_camera;
    rclcpp::Time time_prev = last_frame_time;

    int repredict_count = 0;
    //check whether need to update
    if ((index + 1) < imu_msg_q.size())
    {
        for (unsigned int i = index + 1; i < imu_msg_q.size(); i++)
        {
            sensor_msgs::msg::Imu msg = imu_msg_q[i];
            //calculate new state
            //input
            Vec6 u;
            Vec12 n;
            n.setZero(12);
            u[0] = msg.angular_velocity.x;
            u[1] = msg.angular_velocity.x;
            u[2] = msg.angular_velocity.x;
            u[3] = msg.linear_acceleration.x;
            u[4] = msg.linear_acceleration.y;
            u[5] = msg.linear_acceleration.z;

            //predict new state
            Vec15 x_dot;
            x_dot = xdot(x_prev, u, n);
            rclcpp::Time msg_time(msg.header.stamp);
            double dT = (msg_time - time_prev).seconds();
            x_new = x_prev + (dT * x_dot);

            //Calculate F V
            Mat15x15 A = jacobian_A(x_prev, u, n);
            Mat15x12 U = jacobian_U(x_prev, u, n);
            Mat15x15 I15x15;
            I15x15.setIdentity();
            Mat15x15 F = I15x15 + dT * A;
            Mat15x12 V = dT * U;

            //Calculate cov matrix
            cov_new = F * cov_prev * F.transpose() + V * Q * V.transpose();

            x_prev = x_new;
            cov_prev = cov_new;
            state_q[i] = x_new;
            covariance_q[i] = cov_new;
            time_prev = rclcpp::Time(imu_msg_q[i].header.stamp);
            repredict_count++;
        }
    }
}

Vec6 votoz(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    Vec6 z;
    Affine3d tf_WI;
    geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;
    geometry_msgs::msg::Point t = msg->pose.pose.position;
    /* Tag to camera */
    tf_WI.setIdentity();
    tf_WI.translation() = Vector3d(t.x, t.y, t.z);
    tf_WI.linear() = Matrix3d(Quaterniond(q.w, q.x, q.y, q.z));


    Matrix3d r_WI = tf_WI.linear();
    Vector3d t_WI = tf_WI.translation();

    nav_msgs::msg::Odometry odom_world;
    Vector3d p_world;
    Quaterniond q_world;
    p_world = t_WI;
    q_world = r_WI;
    odom_world.header.stamp = msg->header.stamp;
    odom_world.header.frame_id = "world";
    odom_world.pose.pose.position.x = p_world(0);
    odom_world.pose.pose.position.y = p_world(1);
    odom_world.pose.pose.position.z = p_world(2);
    odom_world.pose.pose.orientation.w = q_world.w();
    odom_world.pose.pose.orientation.x = q_world.x();
    odom_world.pose.pose.orientation.y = q_world.y();
    odom_world.pose.pose.orientation.z = q_world.z();
    pub_odom_world->publish(odom_world);

    Vector3d euler = euler_from_rotation_matrix(r_WI);
    z << t_WI(0), t_WI(1), t_WI(2), euler(1), euler(1), euler(2);
    return z;
}


void odom_callback_vo(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    rclcpp::Time t1 = rclcpp::Clock().now();
    rclcpp::Time t2;
    if (msg->pose.pose.position.x == 0.012345)
    {
        return;
    }
    //creat z_pnp and zflow
    Vec6 z_vo;
    z_vo = votoz(msg);

    if ((!initialized))
    {
        Vec15 x_init;
        Mat15x15 cov_init;
        x_init.setZero();
        x_init.head(6) = z_vo;                      //init the state
        cov_init = 0.1 * MatrixXd::Identity(15, 15); //init the cov
        last_frame_time = rclcpp::Time(msg->header.stamp);
        state_q.push_back(x_init);
        covariance_q.push_back(cov_init);
        initialized = 1;
        cout << x_init << endl;
        cout << "the init state is using vo at:" << rclcpp::Time(msg->header.stamp).seconds() << endl;
        return;
    }

    Vec15 state_prev;
    Mat15x15 covariance_prev;
    Vec15 state_new;
    Mat15x15 covariance_new;

    // sensor_msgs::msg::Imu imu_msg;
    //    cout << "pnp_frame_time:   " << pnp_msg->header.stamp << endl;
    //    cout << "imu_msg_q_size: " << imu_msg_q.size() << endl;
    //    cout << "---q_head_time:   " << imu_msg_q.front().header.stamp << endl;
    //    cout << "---q_end_time :   " << imu_msg_q.back().header.stamp << endl;
    unsigned int index = 0; //index of the closed in the q
    if (imu_msg_q.size() == 0)
        return;

    //cout << (imu_msg_q.front().header.stamp - pnp_msg->header.stamp) << endl;
    if (rclcpp::Time(imu_msg_q.front().header.stamp) > rclcpp::Time(msg->header.stamp))
    {
        cout << "break for time synce" << endl;
        return;
    }
    for (unsigned int i = 0; i < imu_msg_q.size(); i++)
    {
        if ((rclcpp::Time(msg->header.stamp) - rclcpp::Time(imu_msg_q[i].header.stamp)).seconds() > 0.0)
            index = i;
        else
            break;
    }
    //    cout << "pnp in imu q index : " << index << endl;
    state_prev = state_q[index];
    covariance_prev = covariance_q[index];
    Vec6 g;
    Vec6 v;
    v << 0, 0, 0, 0, 0, 0;
    Mat6x15 C = jacobian_C_vo(state_prev);
    Mat6x6 W = jacobian_W_vo(state_prev, v);
    Mat15x6 K;
    g << C * state_prev;
    Vec6 zvo_g = z_vo-g;
    while(abs(zvo_g(5))>2)
    {
        if(zvo_g(5)>0)
        {
            zvo_g(5) -= 3.141592653589793;
        }
        else
        {
            zvo_g(5) += 3.141592653589793;
        }
    }

    K = covariance_prev * C.transpose() * ((C * covariance_prev * C.transpose() + W * R_vo * W.transpose()).inverse());
    state_new = state_prev + K * (zvo_g);
    covariance_new = covariance_prev - K * C * covariance_prev;
    repredict(state_new, covariance_new, rclcpp::Time(msg->header.stamp), index);
    t2 = rclcpp::Clock().now();
    //    cout << "Cost: " << (t2 - t1).seconds() * 1000 << "ms" << endl;
}






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ekf");
    auto s1 = node->create_subscription<sensor_msgs::msg::Imu>("imu", 20, imu_callback);
    auto s2 = node->create_subscription<nav_msgs::msg::Odometry>("vo", 1, odom_callback_vo);

    pub_vel = node->create_publisher<visualization_msgs::msg::Marker>("/ekf_sub/velocity", 1);
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 10);
    pub_odom_world = node->create_publisher<nav_msgs::msg::Odometry>("pnp_odom_world", 10);
    pub_array = node->create_publisher<std_msgs::msg::Float64MultiArray>("plotpqv", 10);

    // You should also tune these parameters
    double ng, na, nbg, nba, n_vo_p, n_vo_q;
    // Q imu covariance matrix;
    node->declare_parameter("ekf.ng", 0.0);
    node->declare_parameter("ekf.na", 0.0);
    node->declare_parameter("ekf.nbg", 0.0);
    node->declare_parameter("ekf.nba", 0.0);
    node->declare_parameter("ekf.vo_p", 0.0);
    node->declare_parameter("ekf.vo_q", 0.0);
    node->get_parameter("ekf.ng", ng);
    node->get_parameter("ekf.na", na);
    node->get_parameter("ekf.nbg", nbg);
    node->get_parameter("ekf.nba", nba);
    node->get_parameter("ekf.vo_p", n_vo_p);
    node->get_parameter("ekf.vo_q", n_vo_q);
    /* optical flow noise */

    cout << "ng     :" << ng << endl;
    cout << "na     :" << na << endl;
    cout << "nbg    :" << nbg << endl;
    cout << "nba    :" << nba << endl;
    cout << "n_vo_p :"  << n_vo_p << endl;
    cout << "n_vo_q :"  << n_vo_q << endl;


    Q.setIdentity();
    R_vo.setIdentity();

    Q.block<3, 3>(0, 0) = ng * Q.block<3, 3>(0, 0);
    Q.block<3, 3>(3, 3) = na * Q.block<3, 3>(0, 0);
    Q.block<3, 3>(6, 6) = nbg * Q.block<3, 3>(6, 6);
    Q.block<3, 3>(9, 9) = nba * Q.block<3, 3>(9, 9);

    //Rt pnp
    R_vo.topLeftCorner(3, 3) = n_vo_p * R_vo.topLeftCorner(3, 3);         //pox_xyz
    R_vo.bottomRightCorner(3, 3) = n_vo_q * R_vo.bottomRightCorner(3, 3); //pitch roll yaw


    cout << "Qt_imu:" << endl
         << Q << endl;
    cout << "R_vo:" << endl
         << R_vo << endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
}
