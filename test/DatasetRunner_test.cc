/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gomez Rodriguez, Jose M.M. Montiel
 * and Juan D. Tardos, University of Zaragoza. Copyright (C) 2014-2016 Raul Mur-Artal, Jose M.M.
 * Montiel and Juan D. Tardos, University of Zaragoza.
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

#include "Common/DatasetRunner.h"
#include <iterator>
#include <gtest/gtest.h>
#include "Common/DatasetRunners.h"

namespace ORB_SLAM3 {

// ─────────────────────────────────────────────────────────────────────────────
// Helper to build argc/argv from a vector of strings
// ─────────────────────────────────────────────────────────────────────────────

class ArgvBuilder {
public:
  explicit ArgvBuilder(std::vector<std::string> args) : _args(std::move(args)) {
    for (auto& a : _args) {
      _ptrs.push_back(a.data());
    }
  }
  [[nodiscard]] int argc() const noexcept {
    return static_cast<int>(_ptrs.size());
  }
  [[nodiscard]] char** argv() noexcept {
    return _ptrs.data();
  }

private:
  std::vector<std::string> _args;
  std::vector<char*>       _ptrs;
};

// ─────────────────────────────────────────────────────────────────────────────
// CLI Parsing Tests
// ─────────────────────────────────────────────────────────────────────────────

class ParseUnifiedTest : public ::testing::Test {
protected:
  // These tests use non-existent paths, so they will fail on file validation.
  // We test the logical validation (dataset/sensor combos) by catching the thrown errors.
};

TEST_F(ParseUnifiedTest, HelpReturnsFalse) {
  ArgvBuilder ab({"prog", "--help"});
  RunConfig   config;
  EXPECT_FALSE(parseUnifiedArguments(ab.argc(), ab.argv(), config));
}

TEST_F(ParseUnifiedTest, MissingDatasetThrows) {
  ArgvBuilder ab({
    "prog",
    "--sensor",
    "mono",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, InvalidDatasetThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "invalid_dataset",
    "--sensor",
    "mono",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, KittiRgbdThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "kitti",
    "--sensor",
    "rgbd",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, KittiInertialThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "kitti",
    "--sensor",
    "mono",
    "--inertial",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, TumStereoThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "tum",
    "--sensor",
    "stereo",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, TumInertialThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "tum",
    "--sensor",
    "mono",
    "--inertial",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, EurocMissingSequencesThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "euroc",
    "--sensor",
    "mono",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

TEST_F(ParseUnifiedTest, TumRgbdMissingAssociationThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "tum",
    "--sensor",
    "rgbd",
    "--vocabulary-file",
    "/tmp/vocab.txt",
    "--settings-file",
    "/tmp/settings.yaml",
    "--sequence-dir",
    "/tmp/data",
  });
  RunConfig   config;
  EXPECT_THROW(parseUnifiedArguments(ab.argc(), ab.argv(), config), std::exception);
}

// ─────────────────────────────────────────────────────────────────────────────
// Factory Tests (RunConfig -> correct runner type + properties)
// ─────────────────────────────────────────────────────────────────────────────

TEST(FactoryTest, EurocMono) {
  RunConfig config;
  config.dataset   = "euroc";
  config.sensor    = "mono";
  config.inertial  = false;
  config.sequences = {"/data/MH01", "timestamps.txt"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::MONOCULAR);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kEuRoC);
  EXPECT_FALSE(runner->useClahe());
}

TEST(FactoryTest, EurocStereoInertial) {
  RunConfig config;
  config.dataset   = "euroc";
  config.sensor    = "stereo";
  config.inertial  = true;
  config.sequences = {"/data/MH01", "timestamps.txt"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::IMU_STEREO);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kEuRoC);
}

TEST(FactoryTest, KittiStereo) {
  RunConfig config;
  config.dataset      = "kitti";
  config.sensor       = "stereo";
  config.sequence_dir = "/data/kitti/00";
  auto runner         = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::STEREO);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kKITTI);
}

TEST(FactoryTest, KittiMono) {
  RunConfig config;
  config.dataset      = "kitti";
  config.sensor       = "mono";
  config.sequence_dir = "/data/kitti/00";
  auto runner         = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::MONOCULAR);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kTUM);
}

TEST(FactoryTest, TumRgbd) {
  RunConfig config;
  config.dataset          = "tum";
  config.sensor           = "rgbd";
  config.sequence_dir     = "/data/tum/fr1";
  config.association_file = "assoc.txt";
  auto runner             = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::RGBD);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kTUM);
}

TEST(FactoryTest, TumMono) {
  RunConfig config;
  config.dataset      = "tum";
  config.sensor       = "mono";
  config.sequence_dir = "/data/tum/fr1";
  auto runner         = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::MONOCULAR);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kTUM);
}

TEST(FactoryTest, TumviMonoInertial) {
  RunConfig config;
  config.dataset   = "tumvi";
  config.sensor    = "mono";
  config.inertial  = true;
  config.sequences = {"/data/room1", "times.txt", "imu.txt"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::IMU_MONOCULAR);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kEuRoC);
  EXPECT_TRUE(runner->useClahe());
  EXPECT_EQ(runner->imreadMode(), cv::IMREAD_GRAYSCALE);
}

TEST(FactoryTest, TumviStereo) {
  RunConfig config;
  config.dataset   = "tumvi";
  config.sensor    = "stereo";
  config.inertial  = false;
  config.sequences = {"/data/left", "/data/right", "times.txt"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->sensorType(), System::STEREO);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kEuRoC);
  EXPECT_TRUE(runner->useClahe());
}

TEST(FactoryTest, UnknownDatasetThrows) {
  RunConfig config;
  config.dataset = "unknown";
  EXPECT_THROW(createDatasetRunner(config), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────────────────────
// IMU Sync Tests (synthetic data injected directly into SequenceData)
// ─────────────────────────────────────────────────────────────────────────────

/// Helper: create a EuRoCRunner with synthetic IMU data injected via load() bypass.
/// We construct the runner, then manually populate _sequences through a test subclass.
class TestableEuRoCRunner : public EuRoCRunner {
public:
  using EuRoCRunner::EuRoCRunner;
  using EuRoCRunner::_sequences;
};

class ImuSyncTest : public ::testing::Test {
protected:
  void SetUp() override {
    RunConfig config;
    config.dataset   = "euroc";
    config.sensor    = "mono";
    config.inertial  = true;
    config.sequences = {"/dummy", "dummy.txt"};
    _runner          = std::make_unique<TestableEuRoCRunner>(config);

    // Inject synthetic data: 5 camera frames, 10 IMU measurements.
    _runner->_sequences.resize(1);
    auto& sd = _runner->_sequences[0];

    // Camera timestamps: 1.0, 2.0, 3.0, 4.0, 5.0
    sd.timestamps = {1.0, 2.0, 3.0, 4.0, 5.0};

    // IMU timestamps: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0
    sd.imu_timestamps = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
    sd.acc.resize(10, cv::Point3f(0.f, 0.f, 9.81f));
    sd.gyro.resize(10, cv::Point3f(0.f, 0.f, 0.f));

    // Compute first_imu: first IMU after first camera timestamp.
    // IMU[0]=0.5 <= 1.0, IMU[1]=1.0 <= 1.0, IMU[2]=1.5 > 1.0 → first_imu=2, then -1 → 1.
    sd.first_imu = 0;
    while (sd.first_imu < std::ssize(sd.imu_timestamps)
           && sd.imu_timestamps[sd.first_imu] <= sd.timestamps[0]) {
      sd.first_imu++;
    }
    if (sd.first_imu > 0) {
      sd.first_imu--;
    }
  }

  std::unique_ptr<TestableEuRoCRunner> _runner;
};

TEST_F(ImuSyncTest, CollectImuAtFrame0ReturnsEmpty) {
  auto imu = _runner->collectImu(0, 0);
  EXPECT_TRUE(imu.empty());
}

TEST_F(ImuSyncTest, CollectImuAtFrame1ReturnsCorrectSlice) {
  // first_imu=1 after SetUp. Collect IMU where t <= 2.0:
  // IMU[1]=1.0 <= 2.0 → collect, IMU[2]=1.5 <= 2.0 → collect, IMU[3]=2.0 <= 2.0 → collect,
  // IMU[4]=2.5 > 2.0 → stop. So 3 measurements, first_imu advances to 4.
  auto imu = _runner->collectImu(0, 1);
  EXPECT_EQ(imu.size(), 3u);
}

TEST_F(ImuSyncTest, CollectImuAdvancesFirstImu) {
  _runner->collectImu(0, 1);
  // After frame 1: first_imu should be 4 (pointing to IMU[4]=2.5).
  auto& sd = _runner->_sequences[0];
  EXPECT_EQ(sd.first_imu, 4);

  // Collect for frame 2 (t=3.0): IMU[4]=2.5 <= 3.0, IMU[5]=3.0 <= 3.0, IMU[6]=3.5 > 3.0.
  auto imu2 = _runner->collectImu(0, 2);
  EXPECT_EQ(imu2.size(), 2u);
  EXPECT_EQ(sd.first_imu, 6);
}

TEST_F(ImuSyncTest, CollectImuAllFramesConsumesAll) {
  // Consume all frames sequentially.
  size_t total = 0;
  for (int ni = 0; ni < 5; ni++) {
    total += _runner->collectImu(0, ni).size();
  }
  // All IMU from index 1 to 9 (those with t <= 5.0) = 9 measurements.
  EXPECT_EQ(total, 9u);
}

TEST_F(ImuSyncTest, CollectImuBoundsCheckPreventsOverread) {
  // Force first_imu to near end and verify no overread.
  auto& sd     = _runner->_sequences[0];
  sd.first_imu = 9;  // Last IMU element.
  // Collect for frame 4 (t=5.0): IMU[9]=5.0 <= 5.0 → collect. first_imu=10 = size, stops.
  auto imu = _runner->collectImu(0, 4);
  EXPECT_EQ(imu.size(), 1u);
  EXPECT_EQ(sd.first_imu, 10);

  // Another call should return empty (bounds check).
  auto imu2 = _runner->collectImu(0, 4);
  EXPECT_TRUE(imu2.empty());
}

TEST_F(ImuSyncTest, FirstImuComputedCorrectly) {
  // Verify first_imu was set to 1 (last IMU index <= camera[0]=1.0).
  EXPECT_EQ(_runner->_sequences[0].first_imu, 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Non-IMU runner collectImu always returns empty
// ─────────────────────────────────────────────────────────────────────────────

TEST(NonImuRunnerTest, KittiCollectImuReturnsEmpty) {
  RunConfig config;
  config.dataset      = "kitti";
  config.sensor       = "stereo";
  config.sequence_dir = "/data/kitti/00";
  auto runner         = createDatasetRunner(config);
  EXPECT_TRUE(runner->collectImu(0, 5).empty());
}

TEST(NonImuRunnerTest, TumCollectImuReturnsEmpty) {
  RunConfig config;
  config.dataset      = "tum";
  config.sensor       = "mono";
  config.sequence_dir = "/data/tum/fr1";
  auto runner         = createDatasetRunner(config);
  EXPECT_TRUE(runner->collectImu(0, 5).empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// Trajectory Format Mapping
// ─────────────────────────────────────────────────────────────────────────────

TEST(TrajectoryFormatTest, EurocUsesEuroc) {
  RunConfig config;
  config.dataset   = "euroc";
  config.sensor    = "stereo";
  config.sequences = {"/data/MH01", "ts.txt"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kEuRoC);
}

TEST(TrajectoryFormatTest, TumviUsesEuroc) {
  RunConfig config;
  config.dataset   = "tumvi";
  config.sensor    = "mono";
  config.sequences = {"/data/room1", "ts.txt"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kEuRoC);
}

TEST(TrajectoryFormatTest, KittiStereoUsesKitti) {
  RunConfig config;
  config.dataset      = "kitti";
  config.sensor       = "stereo";
  config.sequence_dir = "/data/kitti";
  auto runner         = createDatasetRunner(config);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kKITTI);
}

TEST(TrajectoryFormatTest, KittiMonoUsesTum) {
  RunConfig config;
  config.dataset      = "kitti";
  config.sensor       = "mono";
  config.sequence_dir = "/data/kitti";
  auto runner         = createDatasetRunner(config);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kTUM);
}

TEST(TrajectoryFormatTest, TumUsesTum) {
  RunConfig config;
  config.dataset      = "tum";
  config.sensor       = "mono";
  config.sequence_dir = "/data/tum";
  auto runner         = createDatasetRunner(config);
  EXPECT_EQ(runner->trajectoryFormat(), TrajectoryFormat::kTUM);
}

} // namespace ORB_SLAM3
