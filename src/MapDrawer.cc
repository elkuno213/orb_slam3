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

#include "MapDrawer.h"
#include <mutex>
#include <set>
#include <vector>
#include "Atlas.h"
#include "KeyFrame.h"
#include "LoggingUtils.h"
#include "MapPoint.h"
#include "Settings.h"

namespace ORB_SLAM3 {

MapDrawer::MapDrawer(Atlas* pAtlas, const std::string& strSettingPath, Settings* settings)
  : mpAtlas(pAtlas), _logger(logging::CreateModuleLogger("MapDrawer")) {
  if (settings) {
    newParameterLoader(settings);
  } else {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    const bool      is_correct = ParseViewerParamFile(fSettings);

    if (!is_correct) {
      throw std::runtime_error(
        fmt::format("Format incorrect in settings file at {}", strSettingPath)
      );
    }
  }
}

void MapDrawer::newParameterLoader(Settings* settings) {
  mKeyFrameSize      = settings->keyFrameSize();
  mKeyFrameLineWidth = settings->keyFrameLineWidth();
  mGraphLineWidth    = settings->graphLineWidth();
  mPointSize         = settings->pointSize();
  mCameraSize        = settings->cameraSize();
  mCameraLineWidth   = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage& fSettings) {
  bool b_miss_params = false;

  cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
  if (!node.empty()) {
    mKeyFrameSize = node.real();
  } else {
    _logger->error("Viewer.KeyFrameSize parameter doesn't exist or is not a real number*");
    b_miss_params = true;
  }

  node = fSettings["Viewer.KeyFrameLineWidth"];
  if (!node.empty()) {
    mKeyFrameLineWidth = node.real();
  } else {
    _logger->error("Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*");
    b_miss_params = true;
  }

  node = fSettings["Viewer.GraphLineWidth"];
  if (!node.empty()) {
    mGraphLineWidth = node.real();
  } else {
    _logger->error("Viewer.GraphLineWidth parameter doesn't exist or is not a real number*");
    b_miss_params = true;
  }

  node = fSettings["Viewer.PointSize"];
  if (!node.empty()) {
    mPointSize = node.real();
  } else {
    _logger->error("Viewer.PointSize parameter doesn't exist or is not a real number*");
    b_miss_params = true;
  }

  node = fSettings["Viewer.CameraSize"];
  if (!node.empty()) {
    mCameraSize = node.real();
  } else {
    _logger->error("Viewer.CameraSize parameter doesn't exist or is not a real number*");
    b_miss_params = true;
  }

  node = fSettings["Viewer.CameraLineWidth"];
  if (!node.empty()) {
    mCameraLineWidth = node.real();
  } else {
    _logger->error("Viewer.CameraLineWidth parameter doesn't exist or is not a real number*");
    b_miss_params = true;
  }

  return !b_miss_params;
}

void MapDrawer::DrawMapPoints() const {
  Map* pActiveMap = mpAtlas->GetCurrentMap();
  if (!pActiveMap) {
    return;
  }

  const std::vector<MapPoint*>& vpMPs    = pActiveMap->GetAllMapPoints();
  const std::vector<MapPoint*>& vpRefMPs = pActiveMap->GetReferenceMapPoints();

  const std::set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  if (vpMPs.empty()) {
    return;
  }

  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(0.0, 0.0, 0.0);

  for (auto* pMP : vpMPs) {
    if (pMP->isBad() || spRefMPs.contains(pMP)) {
      continue;
    }
    Eigen::Matrix<float, 3, 1> pos = pMP->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }
  glEnd();

  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);

  for (auto* pMP : spRefMPs) {
    if (pMP->isBad()) {
      continue;
    }
    Eigen::Matrix<float, 3, 1> pos = pMP->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }

  glEnd();
}

void MapDrawer::DrawKeyFrames(
  const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba
) {
  const float& w = mKeyFrameSize;
  const float  h = w * 0.75;
  const float  z = w * 0.6;

  Map* pActiveMap = mpAtlas->GetCurrentMap();
  // DEBUG LBA
  std::set<long unsigned int> sOptKFs   = pActiveMap->msOptKFs;
  std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

  if (!pActiveMap) {
    return;
  }

  const std::vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

  if (bDrawKF) {
    for (auto* pKF : vpKFs) {
      Eigen::Matrix4f Twc         = pKF->GetPoseInverse().matrix();
      [[maybe_unused]] const unsigned int index_color = pKF->mnOriginMapId;

      glPushMatrix();

      glMultMatrixf((GLfloat*)Twc.data());

      if (!pKF->GetParent()) // It is the first KF in the map
      {
        glLineWidth(mKeyFrameLineWidth * 5);
        glColor3f(1.0F, 0.0F, 0.0F);
        glBegin(GL_LINES);
      } else {
        glLineWidth(mKeyFrameLineWidth);
        if (bDrawOptLba) {
          if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
            glColor3f(0.0F, 1.0F, 0.0F); // Green -> Opt KFs
          } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
            glColor3f(1.0F, 0.0F, 0.0F); // Red -> Fixed KFs
          } else {
            glColor3f(0.0F, 0.0F, 1.0F); // Basic color
          }
        } else {
          glColor3f(0.0F, 0.0F, 1.0F); // Basic color
        }
        glBegin(GL_LINES);
      }

      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);

      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);

      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);
      glEnd();

      glPopMatrix();

      glEnd();
    }
  }

  if (bDrawGraph) {
    glLineWidth(mGraphLineWidth);
    glColor4f(0.0F, 1.0F, 0.0F, 0.6F);
    glBegin(GL_LINES);

    for (auto* pKF : vpKFs) {
      // Covisibility Graph
      const std::vector<KeyFrame*> vCovKFs = pKF->GetCovisiblesByWeight(100);
      Eigen::Vector3f              Ow      = pKF->GetCameraCenter();
      if (!vCovKFs.empty()) {
        for (auto* pCovKF : vCovKFs) {
          if (pCovKF->mnId < pKF->mnId) {
            continue;
          }
          const Eigen::Vector3f Ow2 = pCovKF->GetCameraCenter();
          glVertex3f(Ow(0), Ow(1), Ow(2));
          glVertex3f(Ow2(0), Ow2(1), Ow2(2));
        }
      }

      // Spanning tree
      KeyFrame* pParent = pKF->GetParent();
      if (pParent) {
        Eigen::Vector3f Owp = pParent->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }

      // Loops
      const std::set<KeyFrame*> sLoopKFs = pKF->GetLoopEdges();
      for (auto* pLoopKF : sLoopKFs) {
        if (pLoopKF->mnId < pKF->mnId) {
          continue;
        }
        Eigen::Vector3f Owl = pLoopKF->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owl(0), Owl(1), Owl(2));
      }
    }

    glEnd();
  }

  if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
    glLineWidth(mGraphLineWidth);
    glColor4f(1.0F, 0.0F, 0.0F, 0.6F);
    glBegin(GL_LINES);

    // Draw inertial links
    for (auto* pKFi : vpKFs) {
      Eigen::Vector3f Ow    = pKFi->GetCameraCenter();
      KeyFrame*       pNext = pKFi->mNextKF;
      if (pNext) {
        Eigen::Vector3f Owp = pNext->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }
    }

    glEnd();
  }

  const std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();

  if (bDrawKF) {
    for (Map* pMap : vpMaps) {
      if (pMap == pActiveMap) {
        continue;
      }

      std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

      for (auto* pKF : vpKFs) {
        Eigen::Matrix4f Twc         = pKF->GetPoseInverse().matrix();
        [[maybe_unused]] const unsigned int index_color = pKF->mnOriginMapId;

        glPushMatrix();

        glMultMatrixf((GLfloat*)Twc.data());

        if (!pKF->GetParent()) // It is the first KF in the map
        {
          glLineWidth(mKeyFrameLineWidth * 5);
          glColor3f(1.0F, 0.0F, 0.0F);
          glBegin(GL_LINES);
        } else {
          glLineWidth(mKeyFrameLineWidth);
          glColor3f(
            mfFrameColors[index_color][0],
            mfFrameColors[index_color][1],
            mfFrameColors[index_color][2]
          );
          glBegin(GL_LINES);
        }

        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
      }
    }
  }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix& Twc) const {
  const float& w = mCameraSize;
  const float  h = w * 0.75;
  const float  z = w * 0.6;

  glPushMatrix();

#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glLineWidth(mCameraLineWidth);
  glColor3f(0.0F, 1.0F, 0.0F);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f& Tcw) {
  const std::unique_lock<std::mutex> lock(mMutexCamera);
  mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(
  pangolin::OpenGlMatrix& M, pangolin::OpenGlMatrix& MOw
) {
  Eigen::Matrix4f Twc;
  {
    const std::unique_lock<std::mutex> lock(mMutexCamera);
    Twc = mCameraPose.matrix();
  }

  for (int i = 0; i < 4; i++) {
    const auto col   = static_cast<ptrdiff_t>(4 * i);
    M.m[col]     = Twc(0, i);
    M.m[col + 1] = Twc(1, i);
    M.m[col + 2] = Twc(2, i);
    M.m[col + 3] = Twc(3, i);
  }

  MOw.SetIdentity();
  MOw.m[12] = Twc(0, 3);
  MOw.m[13] = Twc(1, 3);
  MOw.m[14] = Twc(2, 3);
}
} // namespace ORB_SLAM3
