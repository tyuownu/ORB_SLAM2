#ifndef INTERPOLATION_H
#define INTERPOLATION_H
#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2{

// reference: https://github.com/ros/geometry2/blob/7ea64d25a0631fe91bd277e0d9195f18cc437151/tf2/src/cache.cpp#L176-L196
// TODO: adding slerp(Spherical Linear intERPolation)
class Odom {
  public:
    static bool lTime(const Odom odom1, const Odom odom2) {
        return odom1.GetTime() < odom2.GetTime();
    }

    Odom() : timestamp(0.0), translation(cv::Point3f(0, 0, 0)), yaw(0) {}

    Odom(const double time, const cv::Point3f position, const float angle)
      : timestamp(time), translation(position), yaw(angle) {}

    double GetTime() const { return timestamp; }

    cv::Point3f GetTranslation() const { return translation; }

    void SetTime(const double time) { timestamp = time;};

    void SetTranslation(const cv::Point3f position) { translation = position; }

    void SetYaw(const float angle) { yaw = angle; }

    cv::Mat GetRotation()
    {
        cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32F);
        const float siny = sin(yaw);
        const float cosy = cos(yaw);
        rotationMatrix.at<float>(0, 0) = cosy;
        rotationMatrix.at<float>(0, 1) = -siny;
        rotationMatrix.at<float>(1, 0) = siny;
        rotationMatrix.at<float>(1, 1) = cosy;
        return rotationMatrix;
    }

    float GetYaw() const
    {
        return yaw;
    }
    Odom Interpolate(const Odom other, const double time)
    {
        if (other.GetTime() == timestamp)
        {
            return other;
        }
        // Calculate the ratio
        const double t1 = time - timestamp;
        const double t2 = time - other.GetTime();


        cv::Point3f t;
        t.x = (t1 * other.GetTranslation().x - t2 * translation.x) / (t1 - t2);
        t.y = (t1 * other.GetTranslation().y - t2 * translation.y) / (t1 - t2);
        t.z = (t1 * other.GetTranslation().z - t2 * translation.z) / (t1 - t2);
        float angle = 0.0f;  //  = (t1 * other.GetYaw() - t2 * yaw) / (t1 - t2);
        if (abs(other.GetYaw() - yaw) > M_PI)
        {
            if (other.GetYaw() < 0 && yaw > 0)
                angle = (t1 * other.GetYaw() - t2 * (yaw - 2 * M_PI)) / (t1 - t2);
            else if (other.GetYaw() > 0 && yaw < 0)
                angle = (t1 * other.GetYaw() - t2 * (yaw + 2 * M_PI)) / (t1 - t2);
        }
        else
            angle = (t1 * other.GetYaw() - t2 * yaw) / (t1 - t2);

        if (angle > M_PI)
            angle -= M_PI * 2;

        if (angle < -M_PI)
            angle += M_PI * 2;
        return Odom(time, t, angle);
    }

    cv::Mat GetRT()
    {
        cv::Mat RT = cv::Mat::eye(4, 4, CV_32F);

        (GetRotation()).copyTo(RT.rowRange(0, 3).colRange(0, 3));

        RT.at<float>(0, 3) = translation.x;
        RT.at<float>(1, 3) = translation.y;
        RT.at<float>(2, 3) = translation.z;
        return RT;
    }

    Odom operator-(Odom &other)
    {
        Odom diff;
        float deltaYaw = yaw - other.GetYaw();
        cv::Point3f deltaTranslation = translation - other.GetTranslation();
        // std::cout << __func__ << std::endl;
        // std::cout << "deltaYaw: " << deltaYaw << std::endl;
        // std::cout << "yaw: " << yaw << std::endl;
        // std::cout << "other Yaw: " << other.GetYaw() << std::endl;

        // std::cout << "deltaTranslation: " << deltaTranslation << std::endl;
        // std::cout << "translation: " << translation << std::endl;
        // std::cout << "other translation: " << other.GetTranslation() << std::endl;

        if (deltaYaw > M_PI)
            deltaYaw -= M_PI * 2;

        if (deltaYaw < -M_PI)
            deltaYaw += M_PI * 2;

        const float x = cos(deltaYaw) * deltaTranslation.x - sin(deltaYaw) * deltaTranslation.y;
        const float y = sin(deltaYaw) * deltaTranslation.x + cos(deltaYaw) * deltaTranslation.y;

        // std::cout << "x: " << x << ", y: " << y << std::endl;

        diff.SetYaw(deltaYaw);
        diff.SetTranslation(cv::Point3f(x, y, 0));

        return diff;
    }
  private:
    double timestamp = 0.0;
    cv::Point3f translation = {0, 0, 0};
    float yaw = 0.0f;
};

class OdomInterpolation {
 public:
    // Odom Interpolate(const Odom other, const double time);
    Odom Interpolate(const double time)
    {
        // std::cout << __func__ << ", time: " << time << std::endl;
        sort(vOdom.begin(), vOdom.end(), Odom::lTime);
        Odom output;
        if (vOdom.size() == 0)
        {
            // std::cout << "empty translation list." << std::endl;
            return output;
        }

        if (vOdom[0].GetTime() >= time)
            return vOdom[0];

        if (vOdom[vOdom.size() - 1].GetTime() <= time)
            return vOdom[vOdom.size() - 2].Interpolate(vOdom[vOdom.size() - 1], time);
        for (size_t index = 0; index < vOdom.size(); ++index)
        {
            if (vOdom[index+1].GetTime() >= time)
            {
                output = vOdom[index].Interpolate(vOdom[index+1], time);
                /*
                 *std::cout << std::fixed << std::setprecision(6)
                 *  << "vOdom time: "<< vOdom[index].GetTime()
                 *  << ": " << vOdom[index].GetTranslation() << std::endl;
                 *std::cout << std::fixed << std::setprecision(6)
                 *  << "vOdom time: "<< vOdom[index+1].GetTime()
                 *  << ": " << vOdom[index+1].GetTranslation() << std::endl;
                 */
                break;
            }
        }
        return output;
    }
    void Add(Odom odom)
    {
        vOdom.push_back(odom);
    }

    size_t DataSize() { return vOdom.size(); }
 private:
    std::vector<Odom> vOdom;
};
};
#endif  // end INTERPOLATION_H
