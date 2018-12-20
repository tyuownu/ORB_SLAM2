#include "Param.h"
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2 {
void UBT::loadParam(const std::string &strSettingPath)
{
  cv::FileStorage fSetting(strSettingPath, cv::FileStorage::READ);
  MIN_PARALLAR = fSetting["UBT.MIN_PARALLAR"];
  MIN_MATCHES_NUMBER = fSetting["UBT.MIN_MATCHES_NUMBER"];
  MAX_ITERATIONS = fSetting["UBT.MAX_ITERATIONS"];
  RECONSTRUCT_WITH_H_AND_F = fSetting["UBT.RECONSTRUCT_WITH_H_AND_F"];
  MAX_GOOD_RATIO = fSetting["UBT.MAX_GOOD_RATIO"];
  fSetting.release();
}
UBT ubt;

}
