#ifndef _PARAM_H
#define _PARAM_H
#include <string>

namespace ORB_SLAM2 {

class UBT
{
  public:
    double MIN_PARALLAR = 1.0;
    int MIN_MATCHES_NUMBER = 100;
    int MAX_ITERATIONS = 200;
    int RECONSTRUCT_WITH_H_AND_F = 0;
    double MAX_GOOD_RATIO = 0.9;

    void loadParam(const std::string &strSettingPath);
};
extern UBT ubt;

}

#endif
