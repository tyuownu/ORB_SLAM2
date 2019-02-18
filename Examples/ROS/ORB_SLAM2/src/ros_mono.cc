/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
vector<double> vTimesTrack;
std::ofstream outFile("ros_mono_time.txt");

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

class ImageAndOdomGrabber
{
 public:
  ImageAndOdomGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  void GrabOdom(const nav_msgs::OdometryConstPtr& msg);

  ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings [whether_show_viewer] [whether_save_map]" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,strcmp(argv[3], "false") ? true : false, bool(atoi(argv[4])));

    ImageAndOdomGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber image_sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageAndOdomGrabber::GrabImage,&igb);
    ros::Subscriber odom_sub = nodeHandler.subscribe("/odom", 1, &ImageAndOdomGrabber::GrabOdom,&igb);

    ros::spin();

    SLAM.UpdateScaleUsingAdjacentKeyframe();
    // Stop all threads
    SLAM.Shutdown();
    double totaltime = 0;
    for (size_t i = 0; i < vTimesTrack.size(); ++i)
    {
        totaltime += vTimesTrack[i];
        outFile << i << ": " << vTimesTrack[i] << std::endl;
    }
    sort(vTimesTrack.begin(), vTimesTrack.end());
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;
    outFile << "median: " << vTimesTrack[vTimesTrack.size()/2] << std::endl;
    outFile << "mean: " << totaltime/vTimesTrack.size() << std::endl;
    outFile.close();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
    vTimesTrack.push_back(std::chrono::duration_cast<std::chrono::duration<double> >(t2-t1).count());

}


void ImageAndOdomGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    static cv::Mat image = cv::imread("/home/tyu/dataset/addon.png");
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    /*
    if (image.empty())
    {
        image = cv_ptr->image.clone();
    }
     */
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

  if ((msg->header.seq > 518 && msg->header.seq < 600) || (msg->header.seq > 1518 && msg->header.seq < 1600))
  {
      // cv::Mat image = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
      // cv::imwrite(std::to_string(msg->header.seq)+".png", image);
      mpSLAM->TrackMonocular(image, cv_ptr->header.stamp.toSec());
  }
  else
      mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
    vTimesTrack.push_back(std::chrono::duration_cast<std::chrono::duration<double> >(t2-t1).count());

}
void ImageAndOdomGrabber::GrabOdom(const nav_msgs::OdometryConstPtr& msg)
{
    cv::Point3f position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaternionf quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    tf::Quaternion quat1;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat1);
    double yaw, roll, pitch;
    tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw);
    mpSLAM->AddOdom(msg->header.stamp.toSec(), position, yaw);
}
