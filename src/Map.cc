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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

unsigned long int Map::nNextId = 0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
    mnId = nNextId;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if (mspKeyFrames.count(pKF))
        std::cout << "already have kf " << pKF << ", on level: " << pKF->mnMapId << std::endl;
    mspKeyFrames.insert(pKF);
    pKF->mnMapId = mnId;
    // std::cout << "Map address: " << this << std::endl;
    // std::cout << __func__ << std::endl;
    // std::cout << "keyframe size: " << mspKeyFrames.size() << std::endl;
    // std::cout << "mnMaxKFid = " << mnMaxKFid << ", cur id: " << pKF->mnId << std::endl;
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    pMP->mnMapId = mnId;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    // std::cout << __func__ << std::endl
    // << ", vpMPs size = " << vpMPs.size() << std::endl;
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints(const int level)
{
    unique_lock<mutex> lock(mMutexMap);
    if (level == -1)
    {
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }
    else
    {
        vector<MapPoint*> vKFs;
        for (set<MapPoint*>::iterator it = mspMapPoints.begin(); it != mspMapPoints.end(); it++)
        {
            if (static_cast<unsigned long int>(level) == (*it)->mnMapId)
            {
                vKFs.push_back(*it);
            }
        }
        return vKFs;
    }
}

long unsigned int Map::MapPointsInMap(const int level)
{
    unique_lock<mutex> lock(mMutexMap);
    if (level == -1)
    {
        return mspMapPoints.size();
    }
    else
    {
        int num = 0;
        for (set<MapPoint*>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
        {
            if (static_cast<unsigned long int>(level) == (*sit)->mnMapId)
                num++;
        }
        return num;
    }
}

long unsigned int Map::KeyFramesInMap(const int level)
{
    unique_lock<mutex> lock(mMutexMap);
    if (level == -1)
    {
        return mspKeyFrames.size();
    }
    else
    {
        int num = 0;
        for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
        {
            if (static_cast<unsigned long int>(level) == (*sit)->mnMapId)
                num++;
        }
        return num;
    }
}

void Map::DeleteMapPointsByMapId(const long unsigned int level)
{
    unique_lock<mutex> lock(mMutexMap);
    for (set<MapPoint*>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
    {
        if (level == (*sit)->mnMapId)
        {
            mspMapPoints.erase((*sit));
        }
    }
    // std::cout << __func__ << std::endl;
}

void Map::DeleteKeyFramesByMapId(const long unsigned int level)
{
    unique_lock<mutex> lock(mMutexMap);
    for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
    {
        if (level == (*sit)->mnMapId)
        {
            mspKeyFrames.erase(*sit);
        }
    }
    // std::cout << __func__ << std::endl;
}

void Map::Delete(const long unsigned int level)
{
    DeleteKeyFramesByMapId(level);
    DeleteMapPointsByMapId(level);
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    nNextId = 0;
    // mnId = 0;
}

void Map::TransformToOtherMap(Map* pM)
{
    unique_lock<mutex> lock(mMutexMap);
    pM->mnId = nNextId++;
    std::cout << __func__ << " mapID: " << pM->mnId << std::endl;

    for (set<MapPoint*>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
    {
        (*sit)->SetMap(pM);
        (*sit)->mnMapId = pM->mnId;
        pM->AddMapPoint(*sit);
    }

    for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
    {
        (*sit)->mnMapId = pM->mnId;
        (*sit)->SetMap(pM);
        pM->AddKeyFrame(*sit);
    }
}

template<class Archive>
void Map::serialize(Archive &ar, const unsigned int version)
{
    // donot save mutex
    ar & mspMapPoints;
    ar & mvpKeyFrameOrigins;
    ar & mspKeyFrames;
    ar & mvpReferenceMapPoints;
    ar & mnMaxKFid & mnBigChangeIdx;
}

template void Map::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive&, const unsigned int);

} //namespace ORB_SLAM
