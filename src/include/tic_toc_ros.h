#ifndef TIC_TOC_H
#define TIC_TOC_H

#include <rclcpp/rclcpp.hpp>


//usage:
//tic_toc_ros tt;
//tt.dT_s(); will return the dT in second;
//tt.dT_ms(); will return the dT in mili-second


class tic_toc_ros
{
public:
    tic_toc_ros()
        : clock_(RCL_ROS_TIME)
    {
        tic = clock_.now();
    }

    double dT_s()
    {
        return (clock_.now() - tic).seconds();
    }

    double dT_ms()
    {
        return (clock_.now() - tic).seconds() * 1000.0;
    }

    void toc()
    {
        std::cout << (clock_.now() - tic).seconds() * 1000.0 << "ms" << std::endl;
    }

    void toc(std::string str)
    {
        std::cout << str << " time:" << (clock_.now() - tic).seconds() * 1000.0 << "ms" << std::endl;
    }

private:
    rclcpp::Clock clock_;
    rclcpp::Time tic;
};



#endif // TIC_TOC_H
