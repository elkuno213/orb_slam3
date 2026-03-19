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

#include "KeyFrame.h"
#include <iostream>
#include <list>
#include "Converter.h"
#include "Frame.h"
#include "GeometricCamera.h"
#include "KeyFrameDatabase.h"
#include "LoggingUtils.h"
#include "Map.h"
#include "MapPoint.h"

// #include <boost/serialization/base_object.hpp>
// #include "SerializationUtils.h"

namespace ORB_SLAM3 {

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame()
  : mnFrameId(0)
  , mTimeStamp(0)
  , mnGridCols(kFrameGridCols)
  , mnGridRows(kFrameGridRows)
  , mfGridElementWidthInv(0)
  , mfGridElementHeightInv(0)
  , mnTrackReferenceForFrame(0)
  , mnFuseTargetForKF(0)
  , mnBALocalForKF(0)
  , mnBAFixedForKF(0)
  , mnNumberOfOpt(0)
  , mnLoopQuery(0)
  , mnLoopWords(0)
  , mnRelocQuery(0)
  , mnRelocWords(0)
  , mnMergeQuery(0)
  , mnMergeWords(0)
  , mnPlaceRecognitionQuery(0)
  , mnPlaceRecognitionWords(0)
  , mPlaceRecognitionScore(0)
  , mbCurrentPlaceRecognition(false)
  , mnBAGlobalForKF(0)
  , mnMergeCorrectedForKF(0)
  , mnBALocalForMerge(0)
  , fx(0)
  , fy(0)
  , cx(0)
  , cy(0)
  , invfx(0)
  , invfy(0)
  , mbf(0)
  , mb(0)
  , mThDepth(0)
  , N(0)
  , mvKeys()
  , mvKeysUn()
  , mvuRight()
  , mvDepth()
  , mnScaleLevels(0)
  , mfScaleFactor(0)
  , mfLogScaleFactor(0)
  , mvScaleFactors(0)
  , mvLevelSigma2(0)
  , mvInvLevelSigma2(0)
  , mnMinX(0)
  , mnMinY(0)
  , mnMaxX(0)
  , mnMaxY(0)
  , mPrevKF(nullptr)
  , mNextKF(nullptr)
  , mbHasVelocity(false)
  , mbFirstConnection(true)
  , mpParent(nullptr)
  , mbNotErase(false)
  , mbToBeErased(false)
  , mbBad(false)
  , mHalfBaseline(0)
  , _logger(logging::CreateModuleLogger("KeyFrame"))
  , NLeft(0)
  , NRight(0) {
}

KeyFrame::KeyFrame(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB)
  : bImu(pMap->isImuInitialized())
  , mnFrameId(F.mnId)
  , mTimeStamp(F.mTimeStamp)
  , mnGridCols(kFrameGridCols)
  , mnGridRows(kFrameGridRows)
  , mfGridElementWidthInv(ORB_SLAM3::Frame::mfGridElementWidthInv)
  , mfGridElementHeightInv(ORB_SLAM3::Frame::mfGridElementHeightInv)
  , mnTrackReferenceForFrame(0)
  , mnFuseTargetForKF(0)
  , mnBALocalForKF(0)
  , mnBAFixedForKF(0)
  , mnNumberOfOpt(0)
  , mnLoopQuery(0)
  , mnLoopWords(0)
  , mnRelocQuery(0)
  , mnRelocWords(0)
  , mnPlaceRecognitionQuery(0)
  , mnPlaceRecognitionWords(0)
  , mPlaceRecognitionScore(0)
  , mbCurrentPlaceRecognition(false)
  , mnBAGlobalForKF(0)
  , mnMergeCorrectedForKF(0)
  , mnBALocalForMerge(0)
  , fx(ORB_SLAM3::Frame::fx)
  , fy(ORB_SLAM3::Frame::fy)
  , cx(ORB_SLAM3::Frame::cx)
  , cy(ORB_SLAM3::Frame::cy)
  , invfx(ORB_SLAM3::Frame::invfx)
  , invfy(ORB_SLAM3::Frame::invfy)
  , mbf(F.mbf)
  , mb(F.mb)
  , mThDepth(F.mThDepth)
  , mDistCoef(F.mDistCoef)
  , N(F.N)
  , mvKeys(F.mvKeys)
  , mvKeysUn(F.mvKeysUn)
  , mvuRight(F.mvuRight)
  , mvDepth(F.mvDepth)
  , mDescriptors(F.mDescriptors.clone())
  , mBowVec(F.mBowVec)
  , mFeatVec(F.mFeatVec)
  , mnScaleLevels(F.mnScaleLevels)
  , mfScaleFactor(F.mfScaleFactor)
  , mfLogScaleFactor(F.mfLogScaleFactor)
  , mvScaleFactors(F.mvScaleFactors)
  , mvLevelSigma2(F.mvLevelSigma2)
  , mvInvLevelSigma2(F.mvInvLevelSigma2)
  , mnMinX(ORB_SLAM3::Frame::mnMinX)
  , mnMinY(ORB_SLAM3::Frame::mnMinY)
  , mnMaxX(ORB_SLAM3::Frame::mnMaxX)
  , mnMaxY(ORB_SLAM3::Frame::mnMaxY)
  , mPrevKF(nullptr)
  , mNextKF(nullptr)
  , mpImuPreintegrated(F.mpImuPreintegrated)
  , mImuCalib(F.mImuCalib)
  , mNameFile(F.mNameFile)
  , mnDataset(F.mnDataset)
  , mbHasVelocity(false)
  , mTlr(F.GetRelativePoseTlr())
  , mTrl(F.GetRelativePoseTrl())
  , mvpMapPoints(F.mvpMapPoints)
  , mpKeyFrameDB(pKFDB)
  , mpORBvocabulary(F.mpORBvocabulary)
  , mbFirstConnection(true)
  , mpParent(nullptr)
  , mbNotErase(false)
  , mbToBeErased(false)
  , mbBad(false)
  , mHalfBaseline(F.mb / 2)
  , mpMap(pMap)
  , mK_(F.mK_)
  , _logger(logging::CreateModuleLogger("KeyFrame"))
  , mpCamera(F.mpCamera)
  , mpCamera2(F.mpCamera2)
  , mvLeftToRightMatch(F.mvLeftToRightMatch)
  , mvRightToLeftMatch(F.mvRightToLeftMatch)
  , mvKeysRight(F.mvKeysRight)
  , NLeft(F.Nleft)
  , NRight(F.Nright) {
  mnId = nNextId++;

  mGrid.resize(mnGridCols);
  if (F.Nleft != -1) {
    mGridRight.resize(mnGridCols);
  }
  for (int i = 0; i < mnGridCols; i++) {
    mGrid[i].resize(mnGridRows);
    if (F.Nleft != -1) {
      mGridRight[i].resize(mnGridRows);
    }
    for (int j = 0; j < mnGridRows; j++) {
      mGrid[i][j] = F.mGrid[i][j];
      if (F.Nleft != -1) {
        mGridRight[i][j] = F.mGridRight[i][j];
      }
    }
  }

  if (!F.HasVelocity()) {
    mVw.setZero();
    mbHasVelocity = false;
  } else {
    mVw           = F.GetVelocity();
    mbHasVelocity = true;
  }

  mImuBias = F.mImuBias;
  SetPose(F.GetPose());

  mnOriginMapId = pMap->GetId();
}

void KeyFrame::ComputeBoW() {
  if (mBowVec.empty() || mFeatVec.empty()) {
    const std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
  }
}

void KeyFrame::SetPose(const Sophus::SE3f& Tcw) {
  const std::unique_lock<std::mutex> lock(mMutexPose);

  mTcw = Tcw;
  mRcw = mTcw.rotationMatrix();
  mTwc = mTcw.inverse();
  mRwc = mTwc.rotationMatrix();

  if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
  {
    mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
  }
}

void KeyFrame::SetVelocity(const Eigen::Vector3f& Vw) {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  mVw           = Vw;
  mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::GetImuPose() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTwc * mImuCalib.mTcb;
}

Eigen::Matrix3f KeyFrame::GetRotation() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetVelocity() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mVw;
}

bool KeyFrame::isVelocitySet() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mbHasVelocity;
}

void KeyFrame::AddConnection(KeyFrame* pKF, const int& weight) {
  {
    const std::unique_lock<std::mutex> lock(mMutexConnections);
    if (!mConnectedKeyFrameWeights.contains(pKF)) {
      mConnectedKeyFrameWeights[pKF] = weight;
    } else if (mConnectedKeyFrameWeights[pKF] != weight) {
      mConnectedKeyFrameWeights[pKF] = weight;
    } else {
      return;
    }
  }

  UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
  const std::unique_lock<std::mutex>     lock(mMutexConnections);
  std::vector<std::pair<int, KeyFrame*>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (const auto& [kf, weight] : mConnectedKeyFrameWeights) {
    vPairs.emplace_back(weight, kf);
  }

  std::sort(vPairs.begin(), vPairs.end());
  std::list<KeyFrame*> lKFs;
  std::list<int>       lWs;
  for (const auto& [weight, kf] : vPairs) {
    if (!kf->isBad()) {
      lKFs.push_front(kf);
      lWs.push_front(weight);
    }
  }

  mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(lKFs.begin(), lKFs.end());
  mvOrderedWeights             = std::vector<int>(lWs.begin(), lWs.end());
}

std::set<KeyFrame*> KeyFrame::GetConnectedKeyFrames() {
  const std::unique_lock<std::mutex> lock(mMutexConnections);
  std::set<KeyFrame*>                s;
  for (const auto& [kf, weight] : mConnectedKeyFrameWeights) {
    s.insert(kf);
  }
  return s;
}

std::vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames() {
  const std::unique_lock<std::mutex> lock(mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

std::vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int& N) {
  const std::unique_lock<std::mutex> lock(mMutexConnections);
  if ((int)mvpOrderedConnectedKeyFrames.size() < N) {
    return mvpOrderedConnectedKeyFrames;
  } else {
    return {
      mvpOrderedConnectedKeyFrames.begin(),
      mvpOrderedConnectedKeyFrames.begin() + N
    };
  }
}

std::vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int& w) {
  const std::unique_lock<std::mutex> lock(mMutexConnections);

  if (mvpOrderedConnectedKeyFrames.empty()) {
    return {};
  }

  const auto it
    = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);

  if (it == mvOrderedWeights.end() && mvOrderedWeights.back() < w) {
    return {};
  } else {
    const int n = static_cast<int>(it - mvOrderedWeights.begin());
    return {
      mvpOrderedConnectedKeyFrames.begin(),
      mvpOrderedConnectedKeyFrames.begin() + n
    };
  }
}

int KeyFrame::GetWeight(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.contains(pKF)) {
    return mConnectedKeyFrameWeights[pKF];
  } else {
    return 0;
  }
}

int KeyFrame::GetNumberMPs() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  int                                numberMPs = 0;
  for (const auto* mp : mvpMapPoints) {
    if (mp == nullptr) {
      continue;
    }
    numberMPs++;
  }
  return numberMPs;
}

void KeyFrame::AddMapPoint(MapPoint* pMP, const std::size_t& idx) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const int& idx) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = nullptr;
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP) {
  auto [leftIndex, rightIndex] = pMP->GetIndexInKeyFrame(this);
  if (leftIndex != -1) {
    mvpMapPoints[leftIndex] = nullptr;
  }
  if (rightIndex != -1) {
    mvpMapPoints[rightIndex] = nullptr;
  }
}

void KeyFrame::ReplaceMapPointMatch(const int& idx, MapPoint* pMP) {
  mvpMapPoints[idx] = pMP;
}

std::set<MapPoint*> KeyFrame::GetMapPoints() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  std::set<MapPoint*>                s;
  for (auto* pMP : mvpMapPoints) {
    if (pMP == nullptr) {
      continue;
    }
    if (!pMP->isBad()) {
      s.insert(pMP);
    }
  }
  return s;
}

int KeyFrame::TrackedMapPoints(const int& minObs) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);

  int        nPoints   = 0;
  const bool bCheckObs = minObs > 0;
  for (int i = 0; i < N; i++) {
    MapPoint* pMP = mvpMapPoints[i];
    if (pMP != nullptr) {
      if (!pMP->isBad()) {
        if (bCheckObs) {
          if (mvpMapPoints[i]->Observations() >= minObs) {
            nPoints++;
          }
        } else {
          nPoints++;
        }
      }
    }
  }

  return nPoints;
}

std::vector<MapPoint*> KeyFrame::GetMapPointMatches() {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const std::size_t& idx) {
  const std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent) {
  std::map<KeyFrame*, int> KFcounter;

  std::vector<MapPoint*> vpMP;

  {
    const std::unique_lock<std::mutex> lockMPs(mMutexFeatures);
    vpMP = mvpMapPoints;
  }

  // For all map points in keyframe check in which other keyframes are they seen
  // Increase counter for those keyframes
  for (auto* pMP : vpMP) {
    if (pMP == nullptr) {
      continue;
    }

    if (pMP->isBad()) {
      continue;
    }

    const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();

    for (const auto& [kf, indices] : observations) {
      if (kf->mnId == mnId || kf->isBad() || kf->GetMap() != mpMap) {
        continue;
      }
      KFcounter[kf]++;
    }
  }

  // This should not happen
  if (KFcounter.empty()) {
    return;
  }

  // If the counter is greater than threshold add connection
  // In case no keyframe counter is over threshold add the one with maximum counter
  int       nmax   = 0;
  KeyFrame* pKFmax = nullptr;
  const int th     = 15;

  std::vector<std::pair<int, KeyFrame*>> vPairs;
  vPairs.reserve(KFcounter.size());
  if (!upParent) {
    _logger->debug("UpdateConnections: current key frame {}", mnId);
  }
  for (const auto& [kf, count] : KFcounter) {
    if (!upParent) {
      _logger->debug(
        "UpdateConnections: candidate key frame {} has {} matches",
        kf->mnId,
        count
      );
    }
    if (count > nmax) {
      nmax   = count;
      pKFmax = kf;
    }
    if (count >= th) {
      vPairs.emplace_back(count, kf);
      kf->AddConnection(this, count);
    }
  }

  if (vPairs.empty()) {
    vPairs.emplace_back(nmax, pKFmax);
    pKFmax->AddConnection(this, nmax);
  }

  std::sort(vPairs.begin(), vPairs.end());
  std::list<KeyFrame*> lKFs;
  std::list<int>       lWs;
  for (const auto& [weight, kf] : vPairs) {
    lKFs.push_front(kf);
    lWs.push_front(weight);
  }

  {
    const std::unique_lock<std::mutex> lockCon(mMutexConnections);

    mConnectedKeyFrameWeights    = KFcounter;
    mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(lKFs.begin(), lKFs.end());
    mvOrderedWeights             = std::vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnection && mnId != mpMap->GetInitKFid()) {
      mpParent = mvpOrderedConnectedKeyFrames.front();
      mpParent->AddChild(this);
      mbFirstConnection = false;
    }
  }
}

void KeyFrame::AddChild(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  if (pKF == this) {
    _logger->error("Key frame {} and its own parent {} are the same", mnId, pKF->mnId);
    throw std::invalid_argument("Key frame cannot be its own parent");
  }

  mpParent = pKF;
  pKF->AddChild(this);
}

std::set<KeyFrame*> KeyFrame::GetChilds() {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspChildrens;
}

KeyFrame* KeyFrame::GetParent() {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mpParent;
}

bool KeyFrame::hasChild(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspChildrens.contains(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mbFirstConnection = bFirst;
}

void KeyFrame::AddLoopEdge(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mbNotErase = true;
  mspLoopEdges.insert(pKF);
}

std::set<KeyFrame*> KeyFrame::GetLoopEdges() {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mbNotErase = true;
  mspMergeEdges.insert(pKF);
}

std::set<KeyFrame*> KeyFrame::GetMergeEdges() {
  const std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspMergeEdges;
}

void KeyFrame::SetNotErase() {
  const std::unique_lock<std::mutex> lock(mMutexConnections);
  mbNotErase = true;
}

void KeyFrame::SetErase() {
  {
    const std::unique_lock<std::mutex> lock(mMutexConnections);
    if (mspLoopEdges.empty()) {
      mbNotErase = false;
    }
  }

  if (mbToBeErased) {
    SetBadFlag();
  }
}

void KeyFrame::SetBadFlag() {
  {
    const std::unique_lock<std::mutex> lock(mMutexConnections);
    if (mnId == mpMap->GetInitKFid()) {
      return;
    } else if (mbNotErase) {
      mbToBeErased = true;
      return;
    }
  }

  for (const auto& [kf, weight] : mConnectedKeyFrameWeights) {
    kf->EraseConnection(this);
  }

  for (auto* mp : mvpMapPoints) {
    if (mp != nullptr) {
      mp->EraseObservation(this);
    }
  }

  {
    const std::unique_lock<std::mutex> lock(mMutexConnections);
    const std::unique_lock<std::mutex> lock1(mMutexFeatures);

    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();

    // Update Spanning Tree
    std::set<KeyFrame*> sParentCandidates;
    if (mpParent != nullptr) {
      sParentCandidates.insert(mpParent);
    }

    // Assign at each iteration one children with a parent (the pair with highest covisibility
    // weight) Include that children as new parent candidate for the rest
    while (!mspChildrens.empty()) {
      bool bContinue = false;

      int       max = -1;
      KeyFrame* pC = nullptr;
      KeyFrame* pP = nullptr;

      for (auto* pKF : mspChildrens) {
        if (pKF->isBad()) {
          continue;
        }

        // Check if a parent candidate is connected to the keyframe
        const std::vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
        for (auto* connected : vpConnected) {
          for (auto* candidate : sParentCandidates) {
            if (connected->mnId == candidate->mnId) {
              const int w = pKF->GetWeight(connected);
              if (w > max) {
                pC        = pKF;
                pP        = connected;
                max       = w;
                bContinue = true;
              }
            }
          }
        }
      }

      if (bContinue) {
        pC->ChangeParent(pP);
        sParentCandidates.insert(pC);
        mspChildrens.erase(pC);
      } else {
        break;
      }
    }

    // If a children has no covisibility links with any parent candidate, assign to the original
    // parent of this KF
    if (!mspChildrens.empty()) {
      for (auto* child : mspChildrens) {
        child->ChangeParent(mpParent);
      }
    }

    if (mpParent != nullptr) {
      mpParent->EraseChild(this);
      mTcp = mTcw * mpParent->GetPoseInverse();
    }
    mbBad = true;
  }

  mpMap->EraseKeyFrame(this);
  mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad() {
  const std::unique_lock<std::mutex> lock(mMutexConnections);
  return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF) {
  bool bUpdate = false;
  {
    const std::unique_lock<std::mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.contains(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate = true;
    }
  }

  if (bUpdate) {
    UpdateBestCovisibles();
  }
}

std::vector<std::size_t> KeyFrame::GetFeaturesInArea(
  const float& x, const float& y, const float& r, const bool bRight
) const {
  std::vector<std::size_t> vIndices;
  vIndices.reserve(N);

  const float factorX = r;
  const float factorY = r;

  const int nMinCellX
    = std::max(0, (int)std::floor((x - mnMinX - factorX) * mfGridElementWidthInv));
  if (nMinCellX >= mnGridCols) {
    return vIndices;
  }

  const int nMaxCellX
    = std::min((int)mnGridCols - 1, (int)std::ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
  if (nMaxCellX < 0) {
    return vIndices;
  }

  const int nMinCellY
    = std::max(0, (int)std::floor((y - mnMinY - factorY) * mfGridElementHeightInv));
  if (nMinCellY >= mnGridRows) {
    return vIndices;
  }

  const int nMaxCellY = std::min(
    (int)mnGridRows - 1,
    (int)std::ceil((y - mnMinY + factorY) * mfGridElementHeightInv)
  );
  if (nMaxCellY < 0) {
    return vIndices;
  }

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<std::size_t>& vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
      for (const auto idx : vCell) {
        const cv::KeyPoint& kpUn  = (NLeft == -1) ? mvKeysUn[idx]
                                  : (!bRight)     ? mvKeys[idx]
                                                  : mvKeysRight[idx];
        const float         distx = kpUn.pt.x - x;
        const float         disty = kpUn.pt.y - y;

        if (std::fabs(distx) < r && std::fabs(disty) < r) {
          vIndices.push_back(idx);
        }
      }
    }
  }

  return vIndices;
}

bool KeyFrame::IsInImage(const float& x, const float& y) const {
  return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f& x3D) {
  const float z = mvDepth[i];
  if (z > 0) {
    const float     u = mvKeys[i].pt.x;
    const float     v = mvKeys[i].pt.y;
    const float     x = (u - cx) * z * invfx;
    const float     y = (v - cy) * z * invfy;
    const Eigen::Vector3f x3Dc(x, y, z);

    const std::unique_lock<std::mutex> lock(mMutexPose);
    x3D = mRwc * x3Dc + mTwc.translation();
    return true;
  } else {
    return false;
  }
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  if (N == 0) {
    return -1.0;
  }

  std::vector<MapPoint*> vpMapPoints;
  Eigen::Matrix3f        Rcw;
  Eigen::Vector3f        tcw;
  {
    const std::unique_lock<std::mutex> lock(mMutexFeatures);
    const std::unique_lock<std::mutex> lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    tcw         = mTcw.translation();
    Rcw         = mRcw;
  }

  std::vector<float> vDepths;
  vDepths.reserve(N);
  const Eigen::Matrix<float, 1, 3> Rcw2 = Rcw.row(2);
  const float                      zcw  = tcw(2);
  for (int i = 0; i < N; i++) {
    if (mvpMapPoints[i] != nullptr) {
      MapPoint*             pMP  = mvpMapPoints[i];
      const Eigen::Vector3f x3Dw = pMP->GetWorldPos();
      const float           z    = Rcw2.dot(x3Dw) + zcw;
      vDepths.push_back(z);
    }
  }

  std::sort(vDepths.begin(), vDepths.end());

  return vDepths[(vDepths.size() - 1) / q];
}

void KeyFrame::SetNewBias(const IMU::Bias& b) {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  mImuBias = b;
  if (mpImuPreintegrated != nullptr) {
    mpImuPreintegrated->SetNewBias(b);
  }
}

Eigen::Vector3f KeyFrame::GetGyroBias() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return {mImuBias.bwx, mImuBias.bwy, mImuBias.bwz};
}

Eigen::Vector3f KeyFrame::GetAccBias() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return {mImuBias.bax, mImuBias.bay, mImuBias.baz};
}

IMU::Bias KeyFrame::GetImuBias() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mImuBias;
}

Map* KeyFrame::GetMap() {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap) {
  const std::unique_lock<std::mutex> lock(mMutexMap);
  mpMap = pMap;
}

void KeyFrame::PreSave(
  std::set<KeyFrame*>& spKF, std::set<MapPoint*>& spMP, std::set<GeometricCamera*>& spCam
) {
  // Save the id of each MapPoint in this KF, there can be null pointer in the vector
  mvBackupMapPointsId.clear();
  mvBackupMapPointsId.reserve(N);
  for (int i = 0; i < N; ++i) {
    if (mvpMapPoints[i] != nullptr
        && spMP.find(mvpMapPoints[i]) != spMP.end()) { // Checks if the element is not null
      mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
    } else { // If the element is null his value is -1 because all the id are positives
      mvBackupMapPointsId.push_back(-1);
    }
  }
  // Save the id of each connected KF with it weight
  mBackupConnectedKeyFrameIdWeights.clear();
  for (const auto& [kf, weight] : mConnectedKeyFrameWeights) {
    if (spKF.find(kf) != spKF.end()) {
      mBackupConnectedKeyFrameIdWeights[kf->mnId] = weight;
    }
  }

  // Save the parent id
  mBackupParentId = -1;
  if (mpParent != nullptr && spKF.find(mpParent) != spKF.end()) {
    mBackupParentId = mpParent->mnId;
  }

  // Save the id of the childrens KF
  mvBackupChildrensId.clear();
  mvBackupChildrensId.reserve(mspChildrens.size());
  for (KeyFrame* pKFi : mspChildrens) {
    if (spKF.find(pKFi) != spKF.end()) {
      mvBackupChildrensId.push_back(pKFi->mnId);
    }
  }

  // Save the id of the loop edge KF
  mvBackupLoopEdgesId.clear();
  mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
  for (KeyFrame* pKFi : mspLoopEdges) {
    if (spKF.find(pKFi) != spKF.end()) {
      mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }
  }

  // Save the id of the merge edge KF
  mvBackupMergeEdgesId.clear();
  mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
  for (KeyFrame* pKFi : mspMergeEdges) {
    if (spKF.find(pKFi) != spKF.end()) {
      mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }
  }

  // Camera data
  mnBackupIdCamera = -1;
  if (mpCamera != nullptr && spCam.find(mpCamera) != spCam.end()) {
    mnBackupIdCamera = mpCamera->GetId();
  }

  mnBackupIdCamera2 = -1;
  if (mpCamera2 != nullptr && spCam.find(mpCamera2) != spCam.end()) {
    mnBackupIdCamera2 = mpCamera2->GetId();
  }

  // Inertial data
  mBackupPrevKFId = -1;
  if (mPrevKF != nullptr && spKF.find(mPrevKF) != spKF.end()) {
    mBackupPrevKFId = mPrevKF->mnId;
  }

  mBackupNextKFId = -1;
  if (mNextKF != nullptr && spKF.find(mNextKF) != spKF.end()) {
    mBackupNextKFId = mNextKF->mnId;
  }

  if (mpImuPreintegrated != nullptr) {
    mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
  }
}

void KeyFrame::PostLoad(
  std::map<long unsigned int, KeyFrame*>&   mpKFid,
  std::map<long unsigned int, MapPoint*>&   mpMPid,
  std::map<unsigned int, GeometricCamera*>& mpCamId
) {
  // Rebuild the empty variables

  // Pose
  SetPose(mTcw);

  mTrl = mTlr.inverse();

  // Reference reconstruction
  // Each MapPoint sight from this KeyFrame
  mvpMapPoints.clear();
  mvpMapPoints.resize(N);
  for (int i = 0; i < N; ++i) {
    if (mvBackupMapPointsId[i] != -1) {
      mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
    } else {
      mvpMapPoints[i] = nullptr;
    }
  }

  // Conected KeyFrames with him weight
  mConnectedKeyFrameWeights.clear();
  for (const auto& [id, weight] : mBackupConnectedKeyFrameIdWeights) {
    KeyFrame* pKFi                  = mpKFid[id];
    mConnectedKeyFrameWeights[pKFi] = weight;
  }

  // Restore parent KeyFrame
  if (mBackupParentId >= 0) {
    mpParent = mpKFid[mBackupParentId];
  }

  // KeyFrame childrens
  mspChildrens.clear();
  for (const auto id : mvBackupChildrensId) {
    mspChildrens.insert(mpKFid[id]);
  }

  // Loop edge KeyFrame
  mspLoopEdges.clear();
  for (const auto id : mvBackupLoopEdgesId) {
    mspLoopEdges.insert(mpKFid[id]);
  }

  // Merge edge KeyFrame
  mspMergeEdges.clear();
  for (const auto id : mvBackupMergeEdgesId) {
    mspMergeEdges.insert(mpKFid[id]);
  }

  // Camera data
  if (mnBackupIdCamera >= 0) {
    mpCamera = mpCamId[mnBackupIdCamera];
  } else {
    _logger->error("No main camera found in key frame {}", mnId);
  }
  if (mnBackupIdCamera2 >= 0) {
    mpCamera2 = mpCamId[mnBackupIdCamera2];
  }

  // Inertial data
  if (mBackupPrevKFId != -1) {
    mPrevKF = mpKFid[mBackupPrevKFId];
  }
  if (mBackupNextKFId != -1) {
    mNextKF = mpKFid[mBackupNextKFId];
  }
  mpImuPreintegrated = &mBackupImuPreintegrated;

  // Remove all backup container
  mvBackupMapPointsId.clear();
  mBackupConnectedKeyFrameIdWeights.clear();
  mvBackupChildrensId.clear();
  mvBackupLoopEdgesId.clear();

  UpdateBestCovisibles();
}

bool KeyFrame::ProjectPointDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v) {
  // 3D in absolute coordinates
  const Eigen::Vector3f P = pMP->GetWorldPos();

  // 3D in camera coordinates
  const Eigen::Vector3f Pc  = mRcw * P + mTcw.translation();
  const float&          PcX = Pc(0);
  const float&          PcY = Pc(1);
  const float&          PcZ = Pc(2);

  // Check positive depth
  if (PcZ < 0.0F) {
    _logger->error("Aborted point projection due to negative error {}", PcZ);
    return false;
  }

  // Project in image and check it is not outside
  const float invz = 1.0F / PcZ;
  u                = fx * PcX * invz + cx;
  v                = fy * PcY * invz + cy;

  if (u < mnMinX || u > mnMaxX) {
    return false;
  }
  if (v < mnMinY || v > mnMaxY) {
    return false;
  }

  const float x  = (u - cx) * invfx;
  const float y  = (v - cy) * invfy;
  const float r2 = x * x + y * y;
  const float k1 = mDistCoef.at<float>(0);
  const float k2 = mDistCoef.at<float>(1);
  const float p1 = mDistCoef.at<float>(2);
  const float p2 = mDistCoef.at<float>(3);
  float       k3 = 0;
  if (mDistCoef.total() == 5) {
    k3 = mDistCoef.at<float>(4);
  }

  // Radial distorsion
  float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

  // Tangential distorsion
  x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
  y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

  const float u_distort = x_distort * fx + cx;
  const float v_distort = y_distort * fy + cy;

  u = u_distort;
  v = v_distort;

  kp = cv::Point2f(u, v);

  return true;
}

bool KeyFrame::ProjectPointUnDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v) {
  // 3D in absolute coordinates
  const Eigen::Vector3f P = pMP->GetWorldPos();

  // 3D in camera coordinates
  const Eigen::Vector3f Pc  = mRcw * P + mTcw.translation();
  const float&          PcX = Pc(0);
  const float&          PcY = Pc(1);
  const float&          PcZ = Pc(2);

  // Check positive depth
  if (PcZ < 0.0F) {
    _logger->error("Aborted point projection due to negative error {}", PcZ);
    return false;
  }

  // Project in image and check it is not outside
  const float invz = 1.0F / PcZ;
  u                = fx * PcX * invz + cx;
  v                = fy * PcY * invz + cy;

  if (u < mnMinX || u > mnMaxX) {
    return false;
  }
  if (v < mnMinY || v > mnMaxY) {
    return false;
  }

  kp = cv::Point2f(u, v);

  return true;
}

Sophus::SE3f KeyFrame::GetRelativePoseTrl() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTrl;
}

Sophus::SE3f KeyFrame::GetRelativePoseTlr() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return mTlr;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
  const std::unique_lock<std::mutex> lock(mMutexPose);

  return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
  const std::unique_lock<std::mutex> lock(mMutexPose);

  return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
  const std::unique_lock<std::mutex> lock(mMutexPose);

  return (mTwc * mTlr).translation();
}

Eigen::Matrix<float, 3, 3> KeyFrame::GetRightRotation() {
  const std::unique_lock<std::mutex> lock(mMutexPose);

  return (mTrl.so3() * mTcw.so3()).matrix();
}

Eigen::Vector3f KeyFrame::GetRightTranslation() {
  const std::unique_lock<std::mutex> lock(mMutexPose);
  return (mTrl * mTcw).translation();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc) {
  mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB) {
  mpKeyFrameDB = pKFDB;
}

} // namespace ORB_SLAM3
