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

#include "MapPoint.h"
#include <mutex>
#include "Frame.h"
#include "KeyFrame.h"
#include "LoggingUtils.h"
#include "Map.h"
#include "ORBmatcher.h"

namespace ORB_SLAM3 {

long unsigned int MapPoint::nNextId = 0;
std::mutex        MapPoint::mGlobalMutex;

MapPoint::MapPoint()
  : mnFirstKFid(0)
  , mnFirstFrame(0)
  , nObs(0)
  , mnTrackReferenceForFrame(0)
  , mnLastFrameSeen(0)
  , mnBALocalForKF(0)
  , mnFuseCandidateForKF(0)
  , mnLoopPointForKF(0)
  , mnCorrectedByKF(0)
  , mnCorrectedReference(0)
  , mnBAGlobalForKF(0)
  , mnVisible(1)
  , mnFound(1)
  , mbBad(false)
  , mpReplaced(nullptr)
  , _logger(logging::CreateModuleLogger("MapPoint")) {
  mpReplaced = nullptr;
}

MapPoint::MapPoint(const Eigen::Vector3f& Pos, KeyFrame* pRefKF, Map* pMap)
  : mnFirstKFid(pRefKF->mnId)
  , mnFirstFrame(pRefKF->mnFrameId)
  , nObs(0)
  , mnTrackReferenceForFrame(0)
  , mnLastFrameSeen(0)
  , mnBALocalForKF(0)
  , mnFuseCandidateForKF(0)
  , mnLoopPointForKF(0)
  , mnCorrectedByKF(0)
  , mnCorrectedReference(0)
  , mnBAGlobalForKF(0)
  , mnOriginMapId(pMap->GetId())
  , mpRefKF(pRefKF)
  , mnVisible(1)
  , mnFound(1)
  , mbBad(false)
  , mpReplaced(nullptr)
  , mfMinDistance(0)
  , mfMaxDistance(0)
  , mpMap(pMap)
  , _logger(logging::CreateModuleLogger("MapPoint")) {
  SetWorldPos(Pos);

  mNormalVector.setZero();

  mbTrackInViewR = false;
  mbTrackInView  = false;

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  const std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

MapPoint::MapPoint(
  const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap
)
  : mnFirstKFid(pRefKF->mnId)
  , mnFirstFrame(pRefKF->mnFrameId)
  , nObs(0)
  , mnTrackReferenceForFrame(0)
  , mnLastFrameSeen(0)
  , mnBALocalForKF(0)
  , mnFuseCandidateForKF(0)
  , mnLoopPointForKF(0)
  , mnCorrectedByKF(0)
  , mnCorrectedReference(0)
  , mnBAGlobalForKF(0)
  , mnOriginMapId(pMap->GetId())
  , mpRefKF(pRefKF)
  , mnVisible(1)
  , mnFound(1)
  , mbBad(false)
  , mpReplaced(nullptr)
  , mfMinDistance(0)
  , mfMaxDistance(0)
  , mpMap(pMap)
  , _logger(logging::CreateModuleLogger("MapPoint")) {
  mInvDepth = invDepth;
  mInitU    = (double)uv_init.x;
  mInitV    = (double)uv_init.y;
  mpHostKF  = pHostKF;

  mNormalVector.setZero();

  // Worldpos is not set
  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  const std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

MapPoint::MapPoint(const Eigen::Vector3f& Pos, Map* pMap, Frame* pFrame, const int& idxF)
  : mnFirstKFid(-1)
  , mnFirstFrame(pFrame->mnId)
  , nObs(0)
  , mnTrackReferenceForFrame(0)
  , mnLastFrameSeen(0)
  , mnBALocalForKF(0)
  , mnFuseCandidateForKF(0)
  , mnLoopPointForKF(0)
  , mnCorrectedByKF(0)
  , mnCorrectedReference(0)
  , mnBAGlobalForKF(0)
  , mnOriginMapId(pMap->GetId())
  , mpRefKF(nullptr)
  , mnVisible(1)
  , mnFound(1)
  , mbBad(false)
  , mpReplaced(nullptr)
  , mpMap(pMap)
  , _logger(logging::CreateModuleLogger("MapPoint")) {
  SetWorldPos(Pos);

  Eigen::Vector3f Ow;
  if (pFrame->Nleft == -1 || idxF < pFrame->Nleft) {
    Ow = pFrame->GetCameraCenter();
  } else {
    const Eigen::Matrix3f Rwl = pFrame->GetRwc();
    const Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
    const Eigen::Vector3f twl = pFrame->GetOw();

    Ow = Rwl * tlr + twl;
  }
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector / mNormalVector.norm();

  const Eigen::Vector3f PC               = mWorldPos - Ow;
  const float     dist             = PC.norm();
  const int       level            = (pFrame->Nleft == -1)  ? pFrame->mvKeysUn[idxF].octave
                                   : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                                            : pFrame->mvKeysRight[idxF].octave;
  const float     levelScaleFactor = pFrame->mvScaleFactors[level];
  const int       nLevels          = pFrame->mnScaleLevels;

  mfMaxDistance = dist * levelScaleFactor;
  mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  const std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

void MapPoint::SetWorldPos(const Eigen::Vector3f& Pos) {
  const std::unique_lock<std::mutex> lock2(mGlobalMutex);
  const std::unique_lock<std::mutex> lock(mMutexPos);
  mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos() {
  const std::unique_lock<std::mutex> lock(mMutexPos);
  return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal() {
  const std::unique_lock<std::mutex> lock(mMutexPos);
  return mNormalVector;
}

KeyFrame* MapPoint::GetReferenceKeyFrame() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, int idx) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  std::tuple<int, int>         indexes;

  if (mObservations.contains(pKF)) {
    indexes = mObservations[pKF];
  } else {
    indexes = std::tuple<int, int>(-1, -1);
  }

  if (pKF->NLeft != -1 && idx >= pKF->NLeft) {
    std::get<1>(indexes) = idx;
  } else {
    std::get<0>(indexes) = idx;
  }

  mObservations[pKF] = indexes;

  if (!pKF->mpCamera2 && pKF->mvuRight[idx] >= 0) {
    nObs += 2;
  } else {
    nObs++;
  }
}

void MapPoint::EraseObservation(KeyFrame* pKF) {
  bool bBad = false;
  {
    const std::unique_lock<std::mutex> lock(mMutexFeatures);
    if (mObservations.contains(pKF)) {
      const auto [leftIndex, rightIndex] = mObservations[pKF];

      if (leftIndex != -1) {
        if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0) {
          nObs -= 2;
        } else {
          nObs--;
        }
      }
      if (rightIndex != -1) {
        nObs--;
      }

      mObservations.erase(pKF);

      if (mpRefKF == pKF) {
        mpRefKF = mObservations.begin()->first;
      }

      // If only 2 observations or less, discard point
      if (nObs <= 2) {
        bBad = true;
      }
    }
  }

  if (bBad) {
    SetBadFlag();
  }
}

std::map<KeyFrame*, std::tuple<int, int>> MapPoint::GetObservations() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mObservations;
}

int MapPoint::Observations() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return nObs;
}

void MapPoint::SetBadFlag() {
  std::map<KeyFrame*, std::tuple<int, int>> obs;
  {
    const std::unique_lock<std::mutex> lock1(mMutexFeatures);
    const std::unique_lock<std::mutex> lock2(mMutexPos);
    mbBad = true;
    obs   = mObservations;
    mObservations.clear();
  }
  for (const auto& [pKF, indexes] : obs) {
    const auto [leftIndex, rightIndex] = indexes;
    if (leftIndex != -1) {
      pKF->EraseMapPointMatch(leftIndex);
    }
    if (rightIndex != -1) {
      pKF->EraseMapPointMatch(rightIndex);
    }
  }

  mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced() {
  const std::unique_lock<std::mutex> lock1(mMutexFeatures);
  const std::unique_lock<std::mutex> lock2(mMutexPos);
  return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP) {
  if (pMP->mnId == mnId) {
    return;
  }

  int                                       nvisible = 0;
  int                                       nfound = 0;
  std::map<KeyFrame*, std::tuple<int, int>> obs;
  {
    const std::unique_lock<std::mutex> lock1(mMutexFeatures);
    const std::unique_lock<std::mutex> lock2(mMutexPos);
    obs = mObservations;
    mObservations.clear();
    mbBad      = true;
    nvisible   = mnVisible;
    nfound     = mnFound;
    mpReplaced = pMP;
  }

  for (const auto& [pKF, indexes] : obs) {
    // Replace measurement in keyframe
    const auto [leftIndex, rightIndex] = indexes;

    if (!pMP->IsInKeyFrame(pKF)) {
      if (leftIndex != -1) {
        pKF->ReplaceMapPointMatch(leftIndex, pMP);
        pMP->AddObservation(pKF, leftIndex);
      }
      if (rightIndex != -1) {
        pKF->ReplaceMapPointMatch(rightIndex, pMP);
        pMP->AddObservation(pKF, rightIndex);
      }
    } else {
      if (leftIndex != -1) {
        pKF->EraseMapPointMatch(leftIndex);
      }
      if (rightIndex != -1) {
        pKF->EraseMapPointMatch(rightIndex);
      }
    }
  }
  pMP->IncreaseFound(nfound);
  pMP->IncreaseVisible(nvisible);
  pMP->ComputeDistinctiveDescriptors();

  mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad() {
  std::unique_lock<std::mutex> lock1(mMutexFeatures, std::defer_lock);
  std::unique_lock<std::mutex> lock2(mMutexPos, std::defer_lock);
  std::lock(lock1, lock2);

  return mbBad;
}

void MapPoint::IncreaseVisible(int n) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  mnVisible += n;
}

void MapPoint::IncreaseFound(int n) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  mnFound += n;
}

float MapPoint::GetFoundRatio() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  std::vector<cv::Mat> vDescriptors;

  std::map<KeyFrame*, std::tuple<int, int>> observations;

  {
    const std::unique_lock<std::mutex> lock1(mMutexFeatures);
    if (mbBad) {
      return;
    }
    observations = mObservations;
  }

  if (observations.empty()) {
    return;
  }

  vDescriptors.reserve(observations.size());

  for (const auto& [pKF, indexes] : observations) {
    if (!pKF->isBad()) {
      const auto [leftIndex, rightIndex] = indexes;

      if (leftIndex != -1) {
        vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
      }
      if (rightIndex != -1) {
        vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
      }
    }
  }

  if (vDescriptors.empty()) {
    return;
  }

  // Compute distances between them
  const std::size_t N = vDescriptors.size();

  float Distances[N][N];
  for (std::size_t i = 0; i < N; i++) {
    Distances[i][i] = 0;
    for (std::size_t j = i + 1; j < N; j++) {
      const int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
      Distances[i][j] = distij;
      Distances[j][i] = distij;
    }
  }

  // Take the descriptor with least median distance to the rest
  int BestMedian = INT_MAX;
  int BestIdx    = 0;
  for (std::size_t i = 0; i < N; i++) {
    std::vector<int> vDists(Distances[i], Distances[i] + N);
    sort(vDists.begin(), vDists.end());
    const int median = vDists[0.5 * (N - 1)];

    if (median < BestMedian) {
      BestMedian = median;
      BestIdx    = i;
    }
  }

  {
    const std::unique_lock<std::mutex> lock(mMutexFeatures);
    mDescriptor = vDescriptors[BestIdx].clone();
  }
}

cv::Mat MapPoint::GetDescriptor() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mDescriptor.clone();
}

std::tuple<int, int> MapPoint::GetIndexInKeyFrame(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  if (mObservations.contains(pKF)) {
    return mObservations[pKF];
  } else {
    return {-1, -1};
  }
}

bool MapPoint::IsInKeyFrame(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mObservations.contains(pKF);
}

void MapPoint::UpdateNormalAndDepth() {
  std::map<KeyFrame*, std::tuple<int, int>> observations;
  KeyFrame*                                 pRefKF = nullptr;
  Eigen::Vector3f                           Pos;
  {
    const std::unique_lock<std::mutex> lock1(mMutexFeatures);
    const std::unique_lock<std::mutex> lock2(mMutexPos);
    if (mbBad) {
      return;
    }
    observations = mObservations;
    pRefKF       = mpRefKF;
    Pos          = mWorldPos;
  }

  if (observations.empty()) {
    return;
  }

  Eigen::Vector3f normal;
  normal.setZero();
  int n = 0;
  for (const auto& [pKF, indexes] : observations) {
    const auto [leftIndex, rightIndex] = indexes;

    if (leftIndex != -1) {
      const Eigen::Vector3f Owi     = pKF->GetCameraCenter();
      const Eigen::Vector3f normali = Pos - Owi;
      normal                        = normal + normali / normali.norm();
      n++;
    }
    if (rightIndex != -1) {
      const Eigen::Vector3f Owi     = pKF->GetRightCameraCenter();
      const Eigen::Vector3f normali = Pos - Owi;
      normal                        = normal + normali / normali.norm();
      n++;
    }
  }

  const Eigen::Vector3f PC   = Pos - pRefKF->GetCameraCenter();
  const float           dist = PC.norm();

  const auto [leftIndex, rightIndex] = observations[pRefKF];
  int level = 0;
  if (pRefKF->NLeft == -1) {
    level = pRefKF->mvKeysUn[leftIndex].octave;
  } else if (leftIndex != -1) {
    level = pRefKF->mvKeys[leftIndex].octave;
  } else {
    level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
  }

  // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
  const float levelScaleFactor = pRefKF->mvScaleFactors[level];
  const int   nLevels          = pRefKF->mnScaleLevels;

  {
    const std::unique_lock<std::mutex> lock3(mMutexPos);
    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
    mNormalVector = normal / n;
  }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f& normal) {
  const std::unique_lock<std::mutex> lock3(mMutexPos);
  mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance() {
  const std::unique_lock<std::mutex> lock(mMutexPos);
  return 0.8F * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance() {
  const std::unique_lock<std::mutex> lock(mMutexPos);
  return 1.2F * mfMaxDistance;
}

int MapPoint::PredictScale(const float& currentDist, KeyFrame* pKF) {
  float ratio = 0.0F;
  {
    const std::unique_lock<std::mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = std::ceil(std::log(ratio) / pKF->mfLogScaleFactor);
  if (nScale < 0) {
    nScale = 0;
  } else if (nScale >= pKF->mnScaleLevels) {
    nScale = pKF->mnScaleLevels - 1;
  }

  return nScale;
}

int MapPoint::PredictScale(const float& currentDist, Frame* pF) {
  float ratio = 0.0F;
  {
    const std::unique_lock<std::mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = std::ceil(std::log(ratio) / pF->mfLogScaleFactor);
  if (nScale < 0) {
    nScale = 0;
  } else if (nScale >= pF->mnScaleLevels) {
    nScale = pF->mnScaleLevels - 1;
  }

  return nScale;
}

Map* MapPoint::GetMap() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mpMap;
}

void MapPoint::UpdateMap(Map* pMap) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mpMap = pMap;
}

void MapPoint::PreSave(set<KeyFrame*>& spKF, set<MapPoint*>& spMP) {
  mBackupReplacedId = -1;
  if (mpReplaced != nullptr && spMP.find(mpReplaced) != spMP.end()) {
    mBackupReplacedId = mpReplaced->mnId;
  }

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();
  // Save the id and position in each KF who view it
  for (const auto& [pKFi, indexes] : mObservations) {
    if (spKF.find(pKFi) != spKF.end()) {
      mBackupObservationsId1[pKFi->mnId] = std::get<0>(indexes);
      mBackupObservationsId2[pKFi->mnId] = std::get<1>(indexes);
    } else {
      EraseObservation(pKFi);
    }
  }

  // Save the id of the reference KF
  if (spKF.find(mpRefKF) != spKF.end()) {
    mBackupRefKFId = mpRefKF->mnId;
  }
}

void MapPoint::PostLoad(
  std::map<long unsigned int, KeyFrame*>& mpKFid, std::map<long unsigned int, MapPoint*>& mpMPid
) {
  mpRefKF = mpKFid[mBackupRefKFId];
  if (!mpRefKF) {
    _logger->error("Map point {} observations has no reference key frame", nObs);
  }
  mpReplaced = nullptr;
  if (mBackupReplacedId >= 0) {
    const auto it = mpMPid.find(mBackupReplacedId);
    if (it != mpMPid.end()) {
      mpReplaced = it->second;
    }
  }

  mObservations.clear();

  for (const auto& [id, leftIdx] : mBackupObservationsId1) {
    KeyFrame*  pKFi = mpKFid[id];
    const auto it2  = mBackupObservationsId2.find(id);
    const std::tuple<int, int> indexes = std::tuple<int, int>(leftIdx, it2->second);
    if (pKFi != nullptr) {
      mObservations[pKFi] = indexes;
    }
  }

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();
}

} // namespace ORB_SLAM3
