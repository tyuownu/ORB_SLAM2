#ifndef INTERPOLATION_H
#define INTERPOLATION_H
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2{

// reference: https://github.com/ros/geometry2/blob/7ea64d25a0631fe91bd277e0d9195f18cc437151/tf2/src/cache.cpp#L176-L196
// TODO: adding slerp(Spherical Linear intERPolation)
class Odom {
  public:
    static bool lTime(const Odom odom1, const Odom odom2) {
        return odom1.GetTime() < odom2.GetTime();
    }

    Odom() : dTimestamp(0.0), translation(cv::Point3f(0, 0, 0)) {}

    Odom(const double time, const cv::Point3f position)
      : dTimestamp(time), translation(position) {}

    double GetTime() const { return dTimestamp; }

    cv::Point3f GetTranslation() const { return translation; }

    void SetTime(const double time) { dTimestamp = time;};

    void SetTranslation(const cv::Point3f position) { translation = position; }
    Odom Interpolate(const Odom other, const double time)
    {
        if (other.GetTime() == dTimestamp)
        {
            return other;
        }
        // Calculate the ratio
        const double other_ratio =
          (time - dTimestamp) / (other.GetTime() - dTimestamp);
        const double ratio = 1 - other_ratio;


        cv::Point3f t;
        t.x = ratio * translation.x + other_ratio * other.GetTranslation().x;
        t.y = ratio * translation.y + other_ratio * other.GetTranslation().y;
        t.z = ratio * translation.z + other_ratio * other.GetTranslation().z;
        return Odom(time, t);
    }
  private:
    double dTimestamp = 0.0;
    cv::Point3f translation = {0, 0, 0};
    // Eigen::Quaterniond rotation;
};

class OdomInterpolation {
 public:
    // Odom Interpolate(const Odom other, const double time);
    Odom Interpolate(const double time)
    {
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
            return vOdom[vOdom.size()-1];
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
