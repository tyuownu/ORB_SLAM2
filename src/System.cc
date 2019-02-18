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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>
#include "Param.h"
#include "Interpolation.h"

namespace ORB_SLAM2
{
ofstream ff;

// OdomInterpolation odom;

// reference to: https://github.com/raulmur/evaluate_ate_scale/blob/master/evaluate_ate_scale.py#L51
cv::Mat ComputeRotationMatrixOdom2Plane(const std::vector<cv::Point3f> vPoints, const std::vector<cv::Point3f> vPlanePoints)
{
    if (vPoints.size() == 0 || vPlanePoints.size() == 0 || vPoints.size() != vPlanePoints.size())
    {
        std::cout << "error when compute Rotation matrix." << std::endl;
        return cv::Mat();
    }
    cv::Point3f PointsCenter(0.0f, 0.0f, 0.0f), PlanePointsCenter(0.0f, 0.0f, 0.0f);
    for (unsigned int i = 0; i < vPoints.size(); i++)
    {
        PointsCenter += vPoints[i];
        PlanePointsCenter += vPlanePoints[i];
    }

    PointsCenter = PointsCenter / float(vPoints.size());
    PlanePointsCenter /= float(vPlanePoints.size());
    std::vector<cv::Point3f> vPointsZeroCentered, vPlanePointsZeroCentered;
    vPointsZeroCentered.reserve(vPoints.size());
    vPlanePointsZeroCentered.reserve(vPlanePoints.size());

    // to compute W
    cv::Mat W = cv::Mat::zeros(3, 3, CV_32F);
    for (unsigned int i = 0; i < vPoints.size(); i++)
    {
        vPointsZeroCentered[i] = vPoints[i] - PointsCenter;
        const float x1 = vPointsZeroCentered[i].x;
        const float y1 = vPointsZeroCentered[i].y;
        const float z1 = vPointsZeroCentered[i].z;
        vPlanePointsZeroCentered[i] = vPlanePoints[i] - PlanePointsCenter;
        const float x2 = vPlanePointsZeroCentered[i].x;
        const float y2 = vPlanePointsZeroCentered[i].y;
        const float z2 = vPlanePointsZeroCentered[i].z;
        W += (cv::Mat_<float>(3,3) << x1 * x2, x1 * y2, x1 * z2,
                                      y1 * x2, y1 * y2, y1 * z2,
                                      z1 * x2, z1 * y2, z1 * z2);
    }

    cv::Mat U, d, Vh;
    cv::SVD::compute(W.t(), d, U, Vh);

    return U*Vh;
}

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const bool bSaveMap) :
  mSensor(sensor), mbSaveMap(bSaveMap), mpViewer(static_cast<Viewer*>(NULL)), mbPause(false), mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbIsScaleUpdate(false)
{
    ORB_SLAM2::ubt.loadParam(strSettingsFile);
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode mapfn = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfn.empty())
    {
      mapfile = (string)mapfn;
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false;
    if (has_suffix(strVocFile, ".txt"))
      bVocLoad= mpVocabulary->loadFromTextFile(strVocFile);
    else
      bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << (double)(clock() - tStart)/CLOCKS_PER_SEC <<" seconds."<< endl << endl;

    //Create KeyFrame Database
    //Create the Map

    if (!mapfile.empty() && LoadMap(mapfile))
    {
        bReuseMap = true;
    } else {
        mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
        mpMap = new Map();
    }

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile, bReuseMap);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // load grouth truth
    ifstream f;
    f.open("odom.txt");
    if (!f)
    {
        std::cout << "No odom data." << std::endl;
    } else {
        while (!f.eof())
        {
            string s;
            getline(f, s);
            if (!s.empty())
            {
                stringstream ss;
                ss << s;
                double timestamp;
                ss >> timestamp;
                double x, y, z;
                ss >> x >> y >> z;
                mOdom.Add(Odom(timestamp, cv::Point3f(x, y, z), 0));
            }
        }
        f.close();
    }

    // Test for new function
    /*
    cv::Mat first_xyz = (cv::Mat_<float>(172, 3) << 0.223901000000000,0.0156953000000000,0,
    0.399645000000000,0.00294397000000000,0,
    0.405925000000000,0.00231110000000000,0,
    0.441873000000000,-0.00127582000000000,0,
    0.455021000000000,-0.00256628000000000,0,
    0.482532000000000,-0.00520746000000000,0,
    0.573939000000000,-0.0137220000000000,0,
    0.629693000000000,-0.0188196000000000,0,
    0.789591000000000,-0.0331826000000000,0,
    0.809364000000000,-0.0348968000000000,0,
    0.934198000000000,-0.0458727000000000,0,
    1.06001000000000,-0.0572688000000000,0,
    1.20733000000000,-0.0705604000000000,0,
    1.23719000000000,-0.0732369000000000,0,
    1.29694000000000,-0.0785541000000000,0,
    1.32691000000000,-0.0812194000000000,0,
    1.35678000000000,-0.0838567000000000,0,
    1.49764000000000,-0.0966744000000000,0,
    1.53831000000000,-0.100439000000000,0,
    1.57890000000000,-0.104195000000000,0,
    1.70150000000000,-0.115761000000000,0,
    1.78368000000000,-0.123682000000000,0,
    1.82447000000000,-0.127652000000000,0,
    1.90679000000000,-0.135691000000000,0,
    2.04093000000000,-0.148799000000000,0,
    2.14435000000000,-0.158800000000000,0,
    2.19579000000000,-0.163782000000000,0,
    2.24785000000000,-0.168833000000000,0,
    2.29992000000000,-0.173875000000000,0,
    2.40449000000000,-0.184249000000000,0,
    2.45572000000000,-0.189379000000000,0,
    2.50756000000000,-0.194539000000000,0,
    2.57068000000000,-0.200761000000000,0,
    2.63359000000000,-0.206942000000000,0,
    2.69588000000000,-0.213038000000000,0,
    2.75751000000000,-0.221405000000000,0,
    2.86626000000000,-0.253904000000000,0,
    2.92248000000000,-0.281072000000000,0,
    2.97545000000000,-0.314650000000000,0,
    3.02713000000000,-0.350887000000000,0,
    3.07874000000000,-0.387510000000000,0,
    3.18789000000000,-0.449503000000000,0,
    3.25625000000000,-0.474496000000000,0,
    3.31996000000000,-0.492303000000000,0,
    3.38853000000000,-0.511564000000000,0,
    3.45989000000000,-0.531654000000000,0,
    3.52065000000000,-0.548822000000000,0,
    3.59207000000000,-0.569043000000000,0,
    3.66328000000000,-0.589120000000000,0,
    3.73457000000000,-0.609197000000000,0,
    3.80607000000000,-0.629309000000000,0,
    3.96053000000000,-0.672651000000000,0,
    4.03019000000000,-0.692237000000000,0,
    4.10136000000000,-0.712287000000000,0,
    4.17345000000000,-0.732547000000000,0,
    4.24502000000000,-0.752769000000000,0,
    4.30676000000000,-0.770195000000000,0,
    4.37869000000000,-0.790481000000000,0,
    4.45018000000000,-0.810701000000000,0,
    4.52206000000000,-0.831063000000000,0,
    4.58304000000000,-0.848331000000000,0,
    4.65463000000000,-0.868518000000000,0,
    4.72649000000000,-0.888722000000000,0,
    4.86457000000000,-0.939450000000000,0,
    4.91091000000000,-0.963962000000000,0,
    5.01394000000000,-1.01873000000000,0,
    5.06061000000000,-1.04372000000000,0,
    5.11691000000000,-1.07393000000000,0,
    5.16336000000000,-1.09902000000000,0,
    5.22977000000000,-1.12987000000000,0,
    5.38190000000000,-1.17073000000000,0,
    5.44405000000000,-1.18446000000000,0,
    5.50581000000000,-1.19639000000000,0,
    5.56822000000000,-1.20165000000000,0,
    5.63056000000000,-1.19972000000000,0,
    5.68250000000000,-1.19320000000000,0,
    5.78615000000000,-1.18011000000000,0,
    5.84884000000000,-1.17878000000000,0,
    5.91075000000000,-1.18396000000000,0,
    5.97258000000000,-1.19600000000000,0,
    6.03327000000000,-1.21410000000000,0,
    6.09361000000000,-1.23308000000000,0,
    6.21539000000000,-1.27095000000000,0,
    6.27708000000000,-1.28380000000000,0,
    6.33967000000000,-1.28974000000000,0,
    6.47805000000000,-1.28427000000000,0,
    6.54201000000000,-1.28024000000000,0,
    6.66412000000000,-1.28498000000000,0,
    6.72677000000000,-1.29514000000000,0,
    6.85250000000000,-1.31574000000000,0,
    6.91515000000000,-1.32603000000000,0,
    6.97796000000000,-1.33635000000000,0,
    7.17707000000000,-1.36913000000000,0,
    7.32306000000000,-1.39274000000000,0,
    7.39658000000000,-1.40430000000000,0,
    7.54373000000000,-1.42683000000000,0,
    7.69069000000000,-1.44875000000000,0,
    7.82711000000000,-1.46893000000000,0,
    7.89012000000000,-1.47816000000000,0,
    7.95293000000000,-1.48715000000000,0,
    8.03867000000000,-1.49916000000000,0,
    8.21339000000000,-1.52319000000000,0,
    8.29527000000000,-1.53455000000000,0,
    8.38976000000000,-1.54760000000000,0,
    8.73113000000000,-1.59394000000000,0,
    8.81313000000000,-1.60508000000000,0,
    8.89501000000000,-1.61622000000000,0,
    8.97783000000000,-1.62506000000000,0,
    9.04798000000000,-1.62594000000000,0,
    9.46888000000000,-1.47024000000000,0,
    9.51081000000000,-1.43330000000000,0,
    9.55097000000000,-1.39385000000000,0,
    9.62046000000000,-1.32439000000000,0,
    9.65987000000000,-1.28430000000000,0,
    9.98415000000000,-0.901747000000000,0,
    10.0277000000000,-0.798468000000000,0,
    10.0465000000000,-0.746182000000000,0,
    10.0654000000000,-0.693086000000000,0,
    10.0840000000000,-0.639098000000000,0,
    10.1241000000000,-0.519464000000000,0,
    10.1584000000000,-0.412192000000000,0,
    10.2230000000000,-0.194185000000000,0,
    10.2624000000000,-0.0189376000000000,0,
    10.2662000000000,0.0361617000000000,0,
    10.2656000000000,0.0931824000000000,0,
    10.2306000000000,0.399731000000000,0,
    10.2227000000000,0.456220000000000,0,
    10.2139000000000,0.515337000000000,0,
    10.1926000000000,0.651709000000000,0,
    10.1809000000000,0.721869000000000,0,
    10.1289000000000,1.00406000000000,0,
    10.1147000000000,1.07446000000000,0,
    10.0862000000000,1.21006000000000,0,
    10.0695000000000,1.27707000000000,0,
    10.0453000000000,1.34475000000000,0,
    10.0158000000000,1.40646000000000,0,
    9.97942000000000,1.46634000000000,0,
    9.93715000000000,1.52206000000000,0,
    9.90923000000000,1.55310000000000,0,
    9.83597000000000,1.61899000000000,0,
    9.74333000000000,1.68450000000000,0,
    9.69718000000000,1.71583000000000,0,
    9.66203000000000,1.73914000000000,0,
    9.62486000000000,1.76313000000000,0,
    9.52614000000000,1.81314000000000,0,
    9.47203000000000,1.83260000000000,0,
    9.43137000000000,1.84405000000000,0,
    9.37662000000000,1.85682000000000,0,
    9.32079000000000,1.86943000000000,0,
    9.26508000000000,1.88149000000000,0,
    9.14007000000000,1.90655000000000,0,
    9.07107000000000,1.91913000000000,0,
    9.01544000000000,1.92525000000000,0,
    8.95763000000000,1.92658000000000,0,
    8.90363000000000,1.92399000000000,0,
    8.79024000000000,1.91640000000000,0,
    8.73445000000000,1.91182000000000,0,
    8.67843000000000,1.90729000000000,0,
    8.55323000000000,1.90916000000000,0,
    8.49664000000000,1.91611000000000,0,
    8.42687000000000,1.92717000000000,0,
    8.35650000000000,1.93788000000000,0,
    8.28627000000000,1.94813000000000,0,
    8.21641000000000,1.95793000000000,0,
    8.14497000000000,1.96752000000000,0,
    8.07369000000000,1.97668000000000,0,
    7.93476000000000,1.99346000000000,0,
    7.86175000000000,2.00179000000000,0,
    7.78032000000000,2.01065000000000,0,
    7.70975000000000,2.01813000000000,0,
    7.62554000000000,2.02670000000000,0,
    7.54109000000000,2.03514000000000,0);

    cv::Mat second_xyz = (cv::Mat_<float>(172, 3) << 0,0,0,
    0.0120028730000000,0.00562786360000000,0.0410771370000000,
    0.0122330780000000,0.00558774640000000,0.0416042020000000,
    0.0149859390000000,0.00701302570000000,0.0505211870000000,
    0.0157899090000000,0.00715278970000000,0.0528469050000000,
    0.0185181160000000,0.00915148200000000,0.0602505320000000,
    0.0238537730000000,0.0108618890000000,0.0808803590000000,
    0.0276233520000000,0.0121770320000000,0.0934659990000000,
    0.0374628160000000,0.0173781680000000,0.130538690000000,
    0.0388526690000000,0.0171916300000000,0.134432380000000,
    0.0479585230000000,0.0217154660000000,0.164221760000000,
    0.0565350350000000,0.0251167440000000,0.193682210000000,
    0.0664831700000000,0.0296839400000000,0.227986890000000,
    0.0688768920000000,0.0301086310000000,0.234755960000000,
    0.0724735860000000,0.0317691830000000,0.248582070000000,
    0.0746278020000000,0.0329978240000000,0.256092640000000,
    0.0769240480000000,0.0339153710000000,0.262372550000000,
    0.0863831040000000,0.0388235300000000,0.295509490000000,
    0.0893676730000000,0.0395864360000000,0.304630640000000,
    0.0929791030000000,0.0408274720000000,0.313742790000000,
    0.100956890000000,0.0442660490000000,0.341484280000000,
    0.106716970000000,0.0465373660000000,0.359982520000000,
    0.109371600000000,0.0476421270000000,0.368672130000000,
    0.115081930000000,0.0507993700000000,0.387997540000000,
    0.123650340000000,0.0543608110000000,0.417546990000000,
    0.131184060000000,0.0570541360000000,0.439905290000000,
    0.134500980000000,0.0587296450000000,0.450581880000000,
    0.137970240000000,0.0598136820000000,0.462405380000000,
    0.141156060000000,0.0614389850000000,0.474820260000000,
    0.148644880000000,0.0644881280000000,0.497048020000000,
    0.151799560000000,0.0658132280000000,0.509298150000000,
    0.156174560000000,0.0673013260000000,0.519546930000000,
    0.160670560000000,0.0694017110000000,0.533949080000000,
    0.164727420000000,0.0710203350000000,0.547669530000000,
    0.168825580000000,0.0726335200000000,0.561803580000000,
    0.175915810000000,0.0736706930000000,0.573727730000000,
    0.190898360000000,0.0785268620000000,0.596879480000000,
    0.200014470000000,0.0800548870000000,0.608271180000000,
    0.208590580000000,0.0811933500000000,0.617354270000000,
    0.219133790000000,0.0842809450000000,0.625081060000000,
    0.228406090000000,0.0850251840000000,0.635828970000000,
    0.245872240000000,0.0873789270000000,0.658025800000000,
    0.253665600000000,0.0899570360000000,0.672345940000000,
    0.261380490000000,0.0915057960000000,0.685809370000000,
    0.269622450000000,0.0932451930000000,0.700363280000000,
    0.277678910000000,0.0953229670000000,0.715297580000000,
    0.284300150000000,0.0969190150000000,0.728114960000000,
    0.292149420000000,0.0987422170000000,0.742443320000000,
    0.299476440000000,0.100811030000000,0.757184740000000,
    0.305956660000000,0.103338060000000,0.772169230000000,
    0.314170150000000,0.105098670000000,0.788065080000000,
    0.331907960000000,0.110383410000000,0.821127890000000,
    0.339938700000000,0.112422160000000,0.836151720000000,
    0.347509240000000,0.114255060000000,0.850473520000000,
    0.355507700000000,0.116504010000000,0.865338440000000,
    0.363627850000000,0.119556680000000,0.880826590000000,
    0.372009630000000,0.121128450000000,0.893172140000000,
    0.379314840000000,0.122539590000000,0.908725380000000,
    0.387376100000000,0.125480160000000,0.923685550000000,
    0.394613000000000,0.126767590000000,0.937697770000000,
    0.401752230000000,0.128746750000000,0.948242900000000,
    0.409553650000000,0.130702270000000,0.965491590000000,
    0.416139330000000,0.132874670000000,0.979860660000000,
    0.436795530000000,0.138182680000000,1.00765010000000,
    0.444443640000000,0.139714810000000,1.01677610000000,
    0.462859930000000,0.142785950000000,1.03751810000000,
    0.470440360000000,0.143981750000000,1.04715500000000,
    0.479602040000000,0.146468880000000,1.05726190000000,
    0.486546810000000,0.146464210000000,1.06616330000000,
    0.494973750000000,0.148975210000000,1.07970040000000,
    0.512740490000000,0.154123750000000,1.11250810000000,
    0.519149180000000,0.155752910000000,1.12666290000000,
    0.523388030000000,0.157862660000000,1.14019790000000,
    0.527275800000000,0.159515020000000,1.15478830000000,
    0.529530290000000,0.161811020000000,1.16899160000000,
    0.529794040000000,0.162715670000000,1.17993940000000,
    0.533921600000000,0.165326890000000,1.20372430000000,
    0.537196930000000,0.167670800000000,1.21788180000000,
    0.542523210000000,0.169261440000000,1.23062980000000,
    0.549919720000000,0.171035220000000,1.24235370000000,
    0.556786420000000,0.172789370000000,1.25410530000000,
    0.563205480000000,0.174879630000000,1.26524670000000,
    0.575253190000000,0.177366960000000,1.29068760000000,
    0.581955790000000,0.179606840000000,1.30390170000000,
    0.585434380000000,0.180577840000000,1.31847640000000,
    0.590507690000000,0.185133030000000,1.34889200000000,
    0.593401490000000,0.185545920000000,1.36249850000000,
    0.603182380000000,0.189393010000000,1.38993350000000,
    0.608910500000000,0.191755440000000,1.40194920000000,
    0.619777320000000,0.195057750000000,1.42998050000000,
    0.625450490000000,0.196924980000000,1.44263680000000,
    0.631661060000000,0.198032560000000,1.45415040000000,
    0.648858070000000,0.204071510000000,1.49770100000000,
    0.661594570000000,0.207527910000000,1.52961410000000,
    0.668158770000000,0.210259380000000,1.54494580000000,
    0.680831190000000,0.214382250000000,1.57520790000000,
    0.692888620000000,0.218828380000000,1.60616220000000,
    0.703970310000000,0.222454530000000,1.63434360000000,
    0.709561940000000,0.224983770000000,1.64754710000000,
    0.714056020000000,0.227189030000000,1.66162000000000,
    0.720405220000000,0.228887350000000,1.67972800000000,
    0.734582540000000,0.233184130000000,1.71524440000000,
    0.740809560000000,0.235679810000000,1.73190520000000,
    0.746748810000000,0.239451920000000,1.75181390000000,
    0.774322090000000,0.246044440000000,1.82292460000000,
    0.781407650000000,0.246772140000000,1.83973910000000,
    0.788024430000000,0.247677240000000,1.85709440000000,
    0.790920620000000,0.252820850000000,1.87479590000000,
    0.792752030000000,0.250117180000000,1.88716480000000,
    0.775148210000000,0.262982310000000,1.97672280000000,
    0.768688860000000,0.264751850000000,1.98577120000000,
    0.759405730000000,0.267692950000000,1.99679920000000,
    0.749064450000000,0.270688030000000,2.01725580000000,
    0.742904840000000,0.272248860000000,2.02653600000000,
    0.677216110000000,0.282747090000000,2.11147740000000,
    0.656945590000000,0.284282270000000,2.12766580000000,
    0.646256570000000,0.285637440000000,2.13407350000000,
    0.635906820000000,0.285862270000000,2.14142110000000,
    0.624970080000000,0.287628590000000,2.14868760000000,
    0.601806400000000,0.289490760000000,2.16443110000000,
    0.581573780000000,0.291224120000000,2.17761090000000,
    0.538414000000000,0.294928760000000,2.20497160000000,
    0.500908260000000,0.296647250000000,2.22230170000000,
    0.490503490000000,0.297044660000000,2.22616980000000,
    0.478317800000000,0.297678260000000,2.22951440000000,
    0.408697930000000,0.299274330000000,2.24073430000000,
    0.396945300000000,0.298701520000000,2.24312280000000,
    0.384140700000000,0.299221400000000,2.24498460000000,
    0.354308630000000,0.298871670000000,2.24936010000000,
    0.340470430000000,0.298890830000000,2.25087500000000,
    0.277032610000000,0.300442310000000,2.25679470000000,
    0.262236330000000,0.300967160000000,2.25835420000000,
    0.231293410000000,0.301266310000000,2.26072840000000,
    0.215686800000000,0.301447870000000,2.25962880000000,
    0.200803760000000,0.301199560000000,2.25740600000000,
    0.184644520000000,0.300623980000000,2.25400380000000,
    0.169680120000000,0.299878420000000,2.24831150000000,
    0.156564000000000,0.298364100000000,2.24177530000000,
    0.148064610000000,0.297368970000000,2.23753430000000,
    0.131586190000000,0.295377610000000,2.22448850000000,
    0.114899520000000,0.294484410000000,2.20896240000000,
    0.105221750000000,0.293642700000000,2.20077280000000,
    0.0999521020000000,0.292386110000000,2.19503550000000,
    0.0945365430000000,0.291976630000000,2.18843050000000,
    0.0784624810000000,0.288574640000000,2.16855190000000,
    0.0709118250000000,0.287597600000000,2.15901230000000,
    0.0671711560000000,0.286482510000000,2.15078310000000,
    0.0619485970000000,0.285545410000000,2.14282420000000,
    0.0554921030000000,0.283787940000000,2.13158970000000,
    0.0501480700000000,0.282634380000000,2.12164160000000,
    0.0384139420000000,0.279721140000000,2.09959270000000,
    0.0342875720000000,0.277509000000000,2.08696600000000,
    0.0307385920000000,0.276362720000000,2.07645540000000,
    0.0273253020000000,0.275384310000000,2.06585690000000,
    0.0243708790000000,0.274094250000000,2.05590300000000,
    0.0192714480000000,0.271867280000000,2.03410150000000,
    0.0165588630000000,0.270859600000000,2.02361130000000,
    0.0134734060000000,0.268868800000000,2.01333950000000,
    0.00516965990000000,0.266074240000000,1.98986360000000,
        -3.67164610000000e-05,0.264421400000000,1.98022040000000,
        -0.00555598740000000,0.263013330000000,1.96757090000000,
        -0.0114932060000000,0.261261050000000,1.95491410000000,
        -0.0174952750000000,0.259512660000000,1.94246400000000,
        -0.0230502490000000,0.258098420000000,1.93017890000000,
        -0.0291569230000000,0.256633400000000,1.91892310000000,
        -0.0338343980000000,0.254567980000000,1.90569540000000,
        -0.0446053150000000,0.251619990000000,1.88145790000000,
        -0.0501292350000000,0.250013530000000,1.86932360000000,
        -0.0555037860000000,0.247397970000000,1.85370460000000,
        -0.0613397960000000,0.245798100000000,1.84163510000000,
        -0.0680589970000000,0.243937360000000,1.82671240000000,
        -0.0736817720000000,0.241986010000000,1.81135450000000);

    std::vector<cv::Point3f> vodoms, vkeyframes;
    for (int i = 0; i < 172; i++)
    {
        vodoms.push_back(cv::Point3f(first_xyz.row(i)));
        vkeyframes.push_back(cv::Point3f(second_xyz.row(i)));
    }
    mRotationMatrixFromOdom2CurrentPlane = ComputeRotationMatrixOdom2Plane(vkeyframes, vodoms).t();
    */

    ff.open("KF.txt");
    ff << std::fixed << setprecision(7);


}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check pause
    {
    unique_lock<mutex> lock(mMutexPause);
    if (mbPause)
    {
        mCvResume.wait(lock);
        mbPause = false;
    }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check pause
    {
    unique_lock<mutex> lock(mMutexPause);
    if (mbPause)
    {
        mCvResume.wait(lock);
        mbPause = false;
    }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check pause
    {
    unique_lock<mutex> lock(mMutexPause);
    if (mbPause)
    {
        mCvResume.wait(lock);
        mbPause = false;
    }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
    // std::cout << std::fixed << std::setprecision(6) << "time: " << timestamp << std::endl;

    if (mpMap->KeyFramesInMap() > 5)
    {
        if (mpTracker->mState != Tracking::eTrackingState::OK)
            Tcw = PredictPose(timestamp);
        else if (mpMap->KeyFramesInMap(mpMap->mnId) <= 6 && mpMap->mnId != 0)
        {
            Tcw = PredictPose(timestamp);
            if (mpMap->KeyFramesInMap(mpMap->mnId) == 6)
            {
                // to calculate the following submap's scale, submap must have at least 5 keyframes.
                CalculateScaleUsingAdjacentKeyframe(mpMap->mnId);
            }
        }
    }


    if (0)
    {

    if (Tcw.empty())
    {
        std::cout << std::fixed << std::setprecision(6) << "time: " << timestamp << ", Tcw is empty." << std::endl;
        // TODO: we need to make up the Tcw using odom
        /*
        if (mpTracker->mState == Tracking::eTrackingState::LOST || mpMap->KeyFramesInMap() > 5)
        {
            Tcw = PredictPose(timestamp);
        }
         */
    }
    else
    {
        /*
        if (mpTracker->mState != Tracking::eTrackingState::OK && !mLastPose.empty())
            Tcw = PredictPose(timestamp);
            */
        std::cout << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << std::endl;
        std::cout << std::fixed << std::setprecision(6) << "time: " << timestamp << std::endl << "matrix: " << Tcw << std::endl;
        std::cout << "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << std::endl;
        cv::Mat R = Tcw.rowRange(0, 3).colRange(0, 3).clone().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = Tcw.rowRange(0, 3).col(3).clone();
        cv::Mat Ow = -R * t;

        /*
        if (mpTracker->mState == Tracking::eTrackingState::LOST)
        {
            ff << "LOST" << std::endl;
            ff << "R: " << R << std::endl;
            ff << "t: " << t << std::endl;
            ff << "Ow: " << Ow << std::endl;
        }
         */
        ff << setprecision(6) << timestamp << " " << Ow.at<float>(0) << " " << Ow.at<float>(1) << " " << Ow.at<float>(2)
            << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
    }
    }
    /*

    ff << std::endl << "1. -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-" << std::endl;
    ff << "timestamp: " << timestamp << std::endl;
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    ff << "KF size: " << vpKFs.size() << std::endl << std::endl;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        ff << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }
    ff << std::endl << "2. -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-" << std::endl;
     */

    mvFrameTimestamp.push_back(timestamp);
    mLastPose = Tcw.clone();

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

cv::Mat System::PredictPose(const double time)
{
    cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
    // std::cout << "back: " << mvFrameTimestamp.back() << ", time: " << time << std::endl;
    Odom d1 = mOdom.Interpolate(mvFrameTimestamp.back());
    Odom d2 = mOdom.Interpolate(time);
    Odom diff = d2 - d1;

    // std::cout << "diff Rotation: " << diff.GetRotation() << std::endl;
    cv::Mat mLastPoseRwc = mLastPose.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat mLastPoseTwc = - mLastPoseRwc * mLastPose.rowRange(0, 3).col(3);
    // cv::Mat rot = mLastPose.rowRange(0, 3).colRange(0, 3) * diff.GetRotation();
    cv::Mat rot = mLastPoseRwc * diff.GetRotation();
    cv::Mat Rcw = rot.t();
    // std::cout << "mLastPose: " << mLastPose << std::endl;
    // std::cout << "rot" << rot << std::endl;
    (Rcw).copyTo(pose.rowRange(0, 3).colRange(0, 3));

    cv::Mat t = (cv::Mat_<float> (3, 1) << diff.GetTranslation().x,
        diff.GetTranslation().y, diff.GetTranslation().z);
    // std::cout << "t: " << t << std::endl;
    // std::cout << "mrotation odom -> plane: " << mRotationMatrixFromOdom2CurrentPlane << std::endl;
    // That is the vector on the mapping plane
    t = mRotationMatrixFromOdom2CurrentPlane.t() * t;
    // std::cout << "t: " << t << std::endl;

    // we need adding the t to current pose
    // t = mLastPose.rowRange(0, 3).col(3) + diff.GetRotation() * t;
    t = mLastPoseTwc + diff.GetRotation() * t;
    t = - Rcw * t;
    t.copyTo(pose.rowRange(0, 3).col(3));
    // std::cout << "pose: " << pose << std::endl;
    return pose;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Pause()
{
    unique_lock<mutex> lock(mMutexPause);
    mbPause = true;
}

void System::Resume()
{
    unique_lock<mutex> lock(mMutexPause);
    mbPause = false;
    mCvResume.notify_one();
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

    if (mbSaveMap)
        SaveMap(mapfile);
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}
void System::CalculateScaleUsingAdjacentKeyframe(const int level)
{
    mpLocalMapper->InterruptBA();
    std::vector<KeyFrame*> vpKeyFrames = mpMap->GetAllKeyFrames(level);
    sort(vpKeyFrames.begin(), vpKeyFrames.end(), KeyFrame::lId);
    std::vector<MapPoint*> vpMapPoints = mpMap->GetAllMapPoints(level);
    // std::cout << "keyframes size: " << vpKeyFrames.size() << std::endl;
    // std::cout << "mappoints size: " << vpMapPoints.size() << std::endl;

    std::vector<double> vdScales;

    std::vector<cv::Point3f> vKeyframesPosition;
    std::vector<cv::Point3f> vOdomsPosition;

    for (size_t i = 0; i < vpKeyFrames.size() - 1; i++)
    {
        Odom d1 = mOdom.Interpolate(vpKeyFrames[i]->mTimeStamp);
        /*
         *std::cout << std::fixed << std::setprecision(6)
         *  << "d1 time: "<< d1.GetTime()
         *  << ": " << d1.GetTranslation() << std::endl;
         */
        Odom d2 = mOdom.Interpolate(vpKeyFrames[i+1]->mTimeStamp);
        /*
         *std::cout << std::fixed << std::setprecision(6)
         *  << "d2 time: "<< d2.GetTime()
         *  << ": " << d2.GetTranslation() << std::endl;
         */
        vKeyframesPosition.push_back(cv::Point3f(vpKeyFrames[i]->GetCameraCenter()));
        vOdomsPosition.push_back(d1.GetTranslation());
        if (0 == i)
        {
            vKeyframesPosition.push_back(cv::Point3f(vpKeyFrames[i+1]->GetCameraCenter()));
            vOdomsPosition.push_back(d2.GetTranslation());
        }
        // std::cout << vpKeyFrames[i]->GetCameraCenter().t() << std::endl;
        // std::cout << d1.GetTranslation() << std::endl;
        const double odom_L2 = cv::norm(d1.GetTranslation() - d2.GetTranslation());

        const double real_L2 = cv::norm(vpKeyFrames[i]->GetCameraCenter() - vpKeyFrames[i+1]->GetCameraCenter());
        vdScales.push_back(odom_L2 / real_L2);
        // std::cout << "scale part: " << odom_L2 / real_L2 << std::endl;
    }

    mfScale = std::accumulate(std::begin(vdScales), std::end(vdScales), 0.0) / vdScales.size();
    // std::cout << "vdScales size: " << vdScales.size() << std::endl;
    std::cout << "scale: " << mfScale << std::endl;
    cv::Mat firstPose = vpKeyFrames[0]->GetPose();
    cv::Mat firstPoseT = firstPose.rowRange(0, 3).col(3);
    // std::cout << "first pose: " << firstPose << std::endl;

    for (auto it = vpKeyFrames.begin(); it != vpKeyFrames.end(); ++it)
    {
        cv::Mat pose = (*it)->GetPose();
        // std::cout << "kf pose: " << pose << std::endl;
        pose.col(3).rowRange(0,3) = (pose.col(3).rowRange(0, 3) - firstPoseT) * mfScale + firstPoseT;
        (*it)->SetPose(pose);

        if (!(*it)->mTcp.empty())
        {
            cv::Mat t1 = ((*it)->mTcp.rowRange(0, 3).col(3) - firstPoseT) * mfScale + firstPoseT;
            t1.copyTo((*it)->mTcp.rowRange(0, 3).col(3));
        }
    }

    cv::Mat firstCameraCenter = vpKeyFrames[0]->GetCameraCenter();
    // std::cout << "first camera center: " << firstCameraCenter << std::endl;
    for (auto it = vpMapPoints.begin(); it != vpMapPoints.end(); ++it)
    {
        // std::cout << "mp position: " << (*it)->GetWorldPos() << std::endl;
        (*it)->SetWorldPos(((*it)->GetWorldPos() - firstCameraCenter) * mfScale + firstCameraCenter);
        (*it)->UpdateNormalAndDepth();
    }
    mbIsScaleUpdate = true;
    mRotationMatrixFromOdom2CurrentPlane = ComputeRotationMatrixOdom2Plane(vKeyframesPosition, vOdomsPosition);
    // Last pose translation also need update
    // std::cout << "***************************************" << std::endl;
    // std::cout << "mLastPose: " << mLastPose << std::endl;
    cv::Mat t = mLastPose.rowRange(0, 3).col(3) * mfScale;
    // std::cout << "t: " << t << std::endl;
    t.copyTo(mLastPose.rowRange(0, 3).col(3));
    // std::cout << "pose: " << mLastPose << std::endl;
    // std::cout << "---------------------------------------" << std::endl;

    mpLocalMapper->Release();

    // TODO: project to plan[x, y, z] to [x', y', 0], or [x, y, 0] -> [x', y', z']

}

void System::UpdateScaleUsingAdjacentKeyframe()
{
    if (mOdom.DataSize() == 0)
    {
        std::cout << "no odom data." << std::endl;
        return;
    }

    if (mbIsScaleUpdate)
    {
        // std::cout << "The scale is alread adjusted!" << std::endl;
        return;
    }
    CalculateScaleUsingAdjacentKeyframe();
}

void System::UpdateScaleUsingConnectedKeyframes()
{
    if (mOdom.DataSize() == 0)
    {
        std::cout << "no odom." << std::endl;
    }
    if (mbIsScaleUpdate)
    {
        std::cout << "The scale is alread adjusted!" << std::endl;
        return;
    }
    std::vector<KeyFrame*> vpKeyFrames = mpMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMapPoints = mpMap->GetAllMapPoints();

    std::vector<double> vdScales;

    for (auto it = vpKeyFrames.begin(); it != vpKeyFrames.end(); ++it)
    {
        Odom d1 = mOdom.Interpolate((*it)->mTimeStamp);
        // using all connected keyframe
        set<KeyFrame*> spConnectedKeyFrames = (*it)->GetConnectedKeyFrames();
        for (auto cit = spConnectedKeyFrames.begin(); cit != spConnectedKeyFrames.end(); ++ cit)
        {
            Odom d2 = mOdom.Interpolate((*cit)->mTimeStamp);
            const double odom_L2 = cv::norm(d1.GetTranslation() - d2.GetTranslation());
            const double real_L2 = cv::norm((*it)->GetCameraCenter() - (*cit)->GetCameraCenter());
            vdScales.push_back(odom_L2 / real_L2);
        }
    }

    mfScale = std::accumulate(std::begin(vdScales), std::end(vdScales), 0.0) / vdScales.size();
    std::cout << "scale: " << mfScale << std::endl;

    for (auto it = vpKeyFrames.begin(); it != vpKeyFrames.end(); ++it)
    {
        cv::Mat pose = (*it)->GetPose();
        pose.col(3).rowRange(0,3) = pose.col(3).rowRange(0, 3) * mfScale;
        (*it)->SetPose(pose);

        if (!(*it)->mTcp.empty())
        {
            cv::Mat t1 = (*it)->mTcp.rowRange(0, 3).col(3) * mfScale;
            t1.copyTo((*it)->mTcp.rowRange(0, 3).col(3));
        }
    }

    for (auto it = vpMapPoints.begin(); it != vpMapPoints.end(); ++it)
    {
        (*it)->SetWorldPos((*it)->GetWorldPos() * mfScale);
        (*it)->UpdateNormalAndDepth();
    }
    mbIsScaleUpdate = true;
}

void System::AddOdom(const double time, const cv::Point3f position, const float angle)
{
    mOdom.Add(Odom(time, position, angle));
}

void System::SaveMap(const string &filename)
{
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << mapfile << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();
}

bool System::LoadMap(const string &filename)
{
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << ", Create a new one" << std::endl;
        return false;
    }
    std::cout << "Loading Mapfile: " << mapfile << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBVocabulary(mpVocabulary);
    std::cout << " ...done" << std::endl;
    std::cout << "Map Reconstructing" << std::flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    long unsigned int nMaxKFId = 0;
    for (auto it : vpKFs)
    {
        it->SetORBVocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > nMaxKFId)
            nMaxKFId = it->mnFrameId;
    }
    Frame::nNextId = nMaxKFId;
    std::cout << " ...done" << std::endl;
    in.close();
    return true;
}

} //namespace ORB_SLAM
