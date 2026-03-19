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

#include "OptimizableTypes.h"

namespace ORB_SLAM3 {
bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}

void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
  auto*                       vi        = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  const Eigen::Vector3d       xyz_trans = vi->estimate().map(Xw);

  const double x = xyz_trans[0];
  const double y = xyz_trans[1];
  const double z = xyz_trans[2];

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.F, z, -y, 1.F, 0.F, 0.F, -z, 0.F, x, 0.F, 1.F, 0.F, y, -x, 0.F, 0.F, 0.F, 1.F;

  _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;
}

bool EdgeSE3ProjectXYZOnlyPoseToBody::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPoseToBody::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}

void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus() {
  auto*                 vi = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  const g2o::SE3Quat    T_lw(vi->estimate());
  const Eigen::Vector3d X_l = T_lw.map(Xw);
  const Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

  const double x_w = X_l[0];
  const double y_w = X_l[1];
  const double z_w = X_l[2];

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.F, z_w, -y_w, 1.F, 0.F, 0.F, -z_w, 0.F, x_w, 0.F, 1.F, 0.F, y_w, -x_w, 0.F, 0.F,
    0.F, 1.F;

  _jacobianOplusXi = -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
}

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ()
  : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}

void EdgeSE3ProjectXYZ::linearizeOplus() {
  auto*                 vj        = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
  const g2o::SE3Quat    T(vj->estimate());
  auto*                 vi        = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const Eigen::Vector3d xyz       = vi->estimate();
  const Eigen::Vector3d xyz_trans = T.map(xyz);

  const double x = xyz_trans[0];
  const double y = xyz_trans[1];
  const double z = xyz_trans[2];

  auto projectJac = -pCamera->projectJac(xyz_trans);

  _jacobianOplusXi = projectJac * T.rotation().toRotationMatrix();

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.F, z, -y, 1.F, 0.F, 0.F, -z, 0.F, x, 0.F, 1.F, 0.F, y, -x, 0.F, 0.F, 0.F, 1.F;

  _jacobianOplusXj = projectJac * SE3deriv;
}

EdgeSE3ProjectXYZToBody::EdgeSE3ProjectXYZToBody()
  : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZToBody::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}

bool EdgeSE3ProjectXYZToBody::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}

void EdgeSE3ProjectXYZToBody::linearizeOplus() {
  auto*                 vj   = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
  const g2o::SE3Quat    T_lw(vj->estimate());
  const g2o::SE3Quat    T_rw = mTrl * T_lw;
  auto*                 vi   = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const Eigen::Vector3d X_w  = vi->estimate();
  const Eigen::Vector3d X_l  = T_lw.map(X_w);
  const Eigen::Vector3d X_r  = mTrl.map(T_lw.map(X_w));

  _jacobianOplusXi = -pCamera->projectJac(X_r) * T_rw.rotation().toRotationMatrix();

  const double x = X_l[0];
  const double y = X_l[1];
  const double z = X_l[2];

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.F, z, -y, 1.F, 0.F, 0.F, -z, 0.F, x, 0.F, 1.F, 0.F, y, -x, 0.F, 0.F, 0.F, 1.F;

  _jacobianOplusXj = -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
}

VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>() {
  _marginalized = false;
  _fix_scale    = false;
}

bool VertexSim3Expmap::read(std::istream& is) {
  g2o::Vector7d cam2world;
  for (int i = 0; i < 6; i++) {
    is >> cam2world[i];
  }
  is >> cam2world[6];

  float nextParam = 0.F;
  for (size_t i = 0; i < pCamera1->size(); i++) {
    is >> nextParam;
    pCamera1->setParameter(nextParam, i);
  }

  for (size_t i = 0; i < pCamera2->size(); i++) {
    is >> nextParam;
    pCamera2->setParameter(nextParam, i);
  }

  setEstimate(g2o::Sim3(cam2world).inverse());
  return true;
}

bool VertexSim3Expmap::write(std::ostream& os) const {
  const g2o::Sim3 cam2world(estimate().inverse());
  g2o::Vector7d lv = cam2world.log();
  for (int i = 0; i < 7; i++) {
    os << lv[i] << " ";
  }

  for (size_t i = 0; i < pCamera1->size(); i++) {
    os << pCamera1->getParameter(i) << " ";
  }

  for (size_t i = 0; i < pCamera2->size(); i++) {
    os << pCamera2->getParameter(i) << " ";
  }

  return os.good();
}

EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ()
  : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>() {
}

bool EdgeSim3ProjectXYZ::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}

bool EdgeSim3ProjectXYZ::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}

EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ()
  : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>() {
}

bool EdgeInverseSim3ProjectXYZ::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}

bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}

} // namespace ORB_SLAM3
