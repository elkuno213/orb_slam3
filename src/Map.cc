/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel
 * and Juan D. Tardós, University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M.
 * Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Map.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LoggingUtils.h"
#include "MapPoint.h"

namespace ORB_SLAM3 {

long unsigned int Map::nNextId = 0;

Map::Map()
  : mpFirstRegionKF(nullptr)
  , mbFail(false)
  , mbImuInitialized(false)
  , mnMapChange(0)
  , mnMapChangeNotified(0)
  , mnMaxKFid(0)
  , mnBigChangeIdx(0)
  , mIsInUse(false)
  , mHasTumbnail(false)
  , mbIsInertial(false)
  , mbIMU_BA1(false)
  , mbIMU_BA2(false)
  , _logger(logging::CreateModuleLogger("Map")) {
  mnId       = nNextId++;
  mThumbnail = nullptr;
}

Map::Map(int initKFid)
  : mpFirstRegionKF(nullptr)
  , mbFail(false)
  , mbImuInitialized(false)
  , mnMapChange(0)
  , mnMapChangeNotified(0)
  , mnInitKFid(initKFid)
  , mnMaxKFid(initKFid)
  , mnBigChangeIdx(0)
  , mIsInUse(false)
  , mHasTumbnail(false)
  , mbIsInertial(false)
  , mbIMU_BA1(false)
  , mbIMU_BA2(false)
  , _logger(logging::CreateModuleLogger("Map")) {
  mnId       = nNextId++;
  mThumbnail = nullptr;
}

Map::~Map() {
  _logger->info("Destructing by releasing all resources (map points, keyframes, thumbnail, etc.)");

  // TODO: erase all points from memory
  mspMapPoints.clear();

  // TODO: erase all keyframes from memory
  mspKeyFrames.clear();

  if (mThumbnail) {
    delete mThumbnail;
  }
  mThumbnail = nullptr;

  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  if (mspKeyFrames.empty()) {
    _logger->info("First key frame {} | Initial key frame of map {}", pKF->mnId, mnInitKFid);
    mnInitKFid  = pKF->mnId;
    mpKFinitial = pKF;
    mpKFlowerID = pKF;
  }
  mspKeyFrames.insert(pKF);
  _logger->debug("Key frame {} added to map", pKF->mnId);
  if (pKF->mnId > mnMaxKFid) {
    mnMaxKFid = pKF->mnId;
  }
  if (pKF->mnId < mpKFlowerID->mnId) {
    mpKFlowerID = pKF;
  }
}

void Map::AddMapPoint(MapPoint* pMP) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  _logger->debug("Map point {} added to map", pMP->mnId);
  mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mbImuInitialized = true;
}

bool Map::isImuInitialized() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint* pMP) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  _logger->debug("Erasing map point {}...", pMP->mnId);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  _logger->debug("Erasing key frame {}...", pKF->mnId);
  mspKeyFrames.erase(pKF);
  if (mspKeyFrames.size() > 0) {
    if (pKF->mnId == mpKFlowerID->mnId) {
      std::vector<KeyFrame*> vpKFs
        = std::vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
      std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
      mpKFlowerID = vpKFs[0];
    }
  } else {
    mpKFlowerID = 0;
  }

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return {mspKeyFrames.begin(), mspKeyFrames.end()};
}

std::vector<MapPoint*> Map::GetAllMapPoints() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return {mspMapPoints.begin(), mspMapPoints.end()};
}

long unsigned int Map::MapPointsInMap() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

std::vector<MapPoint*> Map::GetReferenceMapPoints() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetId() const {
  return mnId;
}

long unsigned int Map::GetInitKFid() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF() {
  return mpKFinitial;
}

void Map::SetCurrentMap() {
  mIsInUse = true;
}

void Map::SetStoredMap() {
  mIsInUse = false;
}

void Map::clear() {
  _logger->info("Clearing map: releasing all keyframes, map points, and resetting state");

  //    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end();
  //    sit!=send; sit++)
  //        delete *sit;

  for (auto* pKF : mspKeyFrames) {
    pKF->UpdateMap(nullptr);
    //        delete *sit;
  }

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid        = mnInitKFid;
  mbImuInitialized = false;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
  mbIMU_BA1 = false;
  mbIMU_BA2 = false;
}

bool Map::IsInUse() const {
  return mIsInUse;
}

void Map::SetBad() {
  mbBad = true;
}

bool Map::IsBad() const {
  return mbBad;
}

void Map::ApplyScaledRotation(const Sophus::SE3f& T, const float s, const bool bScaledVel) {
  const std::unique_lock<std::mutex> lock(mMutexMap);

  _logger->info("Applying scale of {:.6f} to rotation{}...", s, bScaledVel ? " and velocity" : "");

  // Body position (IMU) of first keyframe is fixed to (0,0,0)
  const auto&           Tyw = T;
  const Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
  const auto&           tyw = Tyw.translation();

  for (auto* pKF : mspKeyFrames) {
    Sophus::SE3f Twc  = pKF->GetPoseInverse();
    Twc.translation() *= s;
    const Sophus::SE3f Tyc  = Tyw * Twc;
    const Sophus::SE3f Tcy  = Tyc.inverse();
    pKF->SetPose(Tcy);
    const Eigen::Vector3f Vw = pKF->GetVelocity();
    if (!bScaledVel) {
      pKF->SetVelocity(Ryw * Vw);
    } else {
      pKF->SetVelocity(Ryw * Vw * s);
    }
  }
  for (auto* pMP : mspMapPoints) {
    pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
    pMP->UpdateNormalAndDepth();
  }
  mnMapChange++;
}

void Map::SetInertialSensor() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mbIsInertial = true;
}

bool Map::IsInertial() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mbIsInertial;
}

void Map::SetIniertialBA1() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mbIMU_BA1 = true;
}

void Map::SetIniertialBA2() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mbIMU_BA1;
}

bool Map::GetIniertialBA2() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId) {
  mnId = nId;
}

unsigned int Map::GetLowerKFID() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  if (mpKFlowerID) {
    return mpKFlowerID->mnId;
  }
  return 0;
}

int Map::GetMapChangeIndex() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mnMapChange;
}

void Map::IncreaseChangeIndex() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mnMapChange++;
}

int Map::GetLastMapChange() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*>& spCams) {
  _logger->info("PreSave: cleaning up observations and backing up map data.");

  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) {
      continue;
    }

    const auto mpObs = pMPi->GetObservations();
    for (const auto& [pKF, obs] : mpObs) {
      if (pKF->GetMap() != this || pKF->isBad()) {
        pMPi->EraseObservation(pKF);
      }
    }
  }

  // Saves the id of KF origins
  mvBackupKeyFrameOriginsId.clear();
  mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
  for (const auto* pKF : mvpKeyFrameOrigins) {
    mvBackupKeyFrameOriginsId.push_back(pKF->mnId);
  }

  // Backup of MapPoints
  mvpBackupMapPoints.clear();
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) {
      continue;
    }

    mvpBackupMapPoints.push_back(pMPi);
    pMPi->PreSave(mspKeyFrames, mspMapPoints);
  }

  // Backup of KeyFrames
  mvpBackupKeyFrames.clear();
  for (KeyFrame* pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad()) {
      continue;
    }

    mvpBackupKeyFrames.push_back(pKFi);
    pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
  }

  mnBackupKFinitialID = -1;
  if (mpKFinitial) {
    mnBackupKFinitialID = mpKFinitial->mnId;
  }

  mnBackupKFlowerID = -1;
  if (mpKFlowerID) {
    mnBackupKFlowerID = mpKFlowerID->mnId;
  }
}

void Map::PostLoad(
  KeyFrameDatabase* pKFDB,
  ORBVocabulary*    pORBVoc /*, std::map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
  std::map<unsigned int, GeometricCamera*>& mpCams
) {
  _logger->info(
    "PostLoad: restoring map from backup (key frames: {}, map points: {})",
    mvpBackupKeyFrames.size(),
    mvpBackupMapPoints.size()
  );

  std::copy(
    mvpBackupMapPoints.begin(),
    mvpBackupMapPoints.end(),
    std::inserter(mspMapPoints, mspMapPoints.begin())
  );
  std::copy(
    mvpBackupKeyFrames.begin(),
    mvpBackupKeyFrames.end(),
    std::inserter(mspKeyFrames, mspKeyFrames.begin())
  );

  std::map<long unsigned int, MapPoint*> mpMapPointId;
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) {
      continue;
    }

    pMPi->UpdateMap(this);
    mpMapPointId[pMPi->mnId] = pMPi;
  }

  std::map<long unsigned int, KeyFrame*> mpKeyFrameId;
  for (KeyFrame* pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad()) {
      continue;
    }

    pKFi->UpdateMap(this);
    pKFi->SetORBVocabulary(pORBVoc);
    pKFi->SetKeyFrameDatabase(pKFDB);
    mpKeyFrameId[pKFi->mnId] = pKFi;
  }

  // References reconstruction between different instances
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) {
      continue;
    }

    pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
  }

  for (KeyFrame* pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad()) {
      continue;
    }

    pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
    pKFDB->add(pKFi);
  }

  if (mnBackupKFinitialID != static_cast<unsigned long int>(-1)) {
    mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
  }

  if (mnBackupKFlowerID != static_cast<unsigned long int>(-1)) {
    mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
  }

  mvpKeyFrameOrigins.clear();
  mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
  for (const auto id : mvBackupKeyFrameOriginsId) {
    mvpKeyFrameOrigins.push_back(mpKeyFrameId[id]);
  }

  mvpBackupMapPoints.clear();
}

} // namespace ORB_SLAM3
