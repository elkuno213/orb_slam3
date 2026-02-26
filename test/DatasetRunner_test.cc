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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include "Common/EuRoCRunner.h"

namespace fs = std::filesystem;

namespace ORB_SLAM3 {

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Helper to build argc/argv from a vector of strings                                             //

class ArgvBuilder {
public:
  explicit ArgvBuilder(std::vector<std::string> args) : _args(std::move(args)) {
    for (auto& a : _args) {
      _ptrs.push_back(a.data());
    }
  }
  [[nodiscard]] int    argc() const noexcept { return static_cast<int>(_ptrs.size()); }
  [[nodiscard]] char** argv() noexcept { return _ptrs.data(); }

private:
  std::vector<std::string> _args;
  std::vector<char*>       _ptrs;
};

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// CLI Parsing Tests                                                                              //

TEST(ParseUnifiedTest, HelpReturnsFalse) {
  ArgvBuilder ab({"prog", "--help"});
  RunConfig   config;
  EXPECT_FALSE(parseArgs(ab.argc(), ab.argv(), config));
}

struct ParseErrorCase {
  std::string              name;
  std::vector<std::string> args;
};

class ParseUnifiedErrorTest : public ::testing::TestWithParam<ParseErrorCase> {};

TEST_P(ParseUnifiedErrorTest, InvalidArgsCombinationThrows) {
  ArgvBuilder ab(GetParam().args);
  RunConfig   config;
  EXPECT_THROW((void)parseArgs(ab.argc(), ab.argv(), config), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  ParseErrors,
  ParseUnifiedErrorTest,
  ::testing::Values(
    ParseErrorCase{
      "MissingDataset",
      {"prog", "--sensor", "mono", "--vocabulary-file", "/tmp/vocab.txt", "--settings-file",
       "/tmp/settings.yaml", "--data", "/tmp/data"}},
    ParseErrorCase{
      "InvalidDataset",
      {"prog", "--dataset", "invalid_dataset", "--sensor", "mono", "--vocabulary-file",
       "/tmp/vocab.txt", "--settings-file", "/tmp/settings.yaml", "--data", "/tmp/data"}},
    ParseErrorCase{
      "TumStereo",
      {"prog", "--dataset", "tum", "--sensor", "stereo", "--vocabulary-file", "/tmp/vocab.txt",
       "--settings-file", "/tmp/settings.yaml", "--data", "/tmp/data"}},
    ParseErrorCase{
      "TumInertial",
      {"prog", "--dataset", "tum", "--sensor", "mono", "--inertial", "--vocabulary-file",
       "/tmp/vocab.txt", "--settings-file", "/tmp/settings.yaml", "--data", "/tmp/data"}},
    ParseErrorCase{
      "MissingData",
      {"prog", "--dataset", "euroc", "--sensor", "mono", "--vocabulary-file", "/tmp/vocab.txt",
       "--settings-file", "/tmp/settings.yaml"}}
  ),
  [](const auto& info) { return info.param.name; }
);

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// CLI Happy-Path Test (uses temp files for file validation)                                      //

class ParseUnifiedHappyPathTest : public ::testing::Test {
protected:
  void SetUp() override {
    _tmp_dir = fs::temp_directory_path() / "orbslam3_test_cli";
    fs::create_directories(_tmp_dir);
    _vocab_path    = _tmp_dir / "vocab.txt";
    _settings_path = _tmp_dir / "settings.yaml";
    std::ofstream(_vocab_path.string()) << "dummy vocab\n";
    std::ofstream(_settings_path.string()) << "dummy settings\n";
  }

  void TearDown() override { fs::remove_all(_tmp_dir); }

  fs::path _tmp_dir;
  fs::path _vocab_path;
  fs::path _settings_path;
};

TEST_F(ParseUnifiedHappyPathTest, EurocStereoInertialParsesCorrectly) {
  ArgvBuilder ab({
    "prog",           "--dataset",    "euroc",
    "--sensor",       "stereo",       "--inertial",
    "--vocabulary-file", _vocab_path.string(),
    "--settings-file",   _settings_path.string(),
    "--output-dir",      _tmp_dir.string(),
    "--data",            "/data/V101", "/data/V102",
  });

  RunConfig config;
  ASSERT_TRUE(parseArgs(ab.argc(), ab.argv(), config));
  EXPECT_EQ(config.dataset, DatasetType::EuRoC);
  EXPECT_EQ(config.sensor, System::IMU_STEREO);
  EXPECT_TRUE(config.use_viewer);
  ASSERT_EQ(config.data_dirs.size(), 2u);
  EXPECT_EQ(config.data_dirs[0], "/data/V101");
  EXPECT_EQ(config.data_dirs[1], "/data/V102");
}

TEST_F(ParseUnifiedHappyPathTest, TumMultipleDataPathsParsesCorrectly) {
  const auto second_dir = _tmp_dir / "seq2";
  fs::create_directories(second_dir);

  ArgvBuilder ab({
    "prog",           "--dataset",    "tum",
    "--sensor",       "mono",
    "--vocabulary-file", _vocab_path.string(),
    "--settings-file",   _settings_path.string(),
    "--output-dir",      _tmp_dir.string(),
    "--no-viewer",
    "--data",            _tmp_dir.string(), second_dir.string(),
  });

  RunConfig config;
  ASSERT_TRUE(parseArgs(ab.argc(), ab.argv(), config));
  EXPECT_EQ(config.dataset, DatasetType::TUM);
  EXPECT_EQ(config.sensor, System::MONOCULAR);
  ASSERT_EQ(config.data_dirs.size(), 2u);
  EXPECT_EQ(config.data_dirs[0], _tmp_dir);
  EXPECT_EQ(config.data_dirs[1], second_dir);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Factory Tests (RunConfig -> correct runner type + properties)                                  //

struct FactoryCase {
  std::string      name;
  DatasetType      dataset;
  System::eSensor  sensor;
  bool             expect_clahe;
  int              expect_imread; // -1 = don't check
};

class FactoryParamTest : public ::testing::TestWithParam<FactoryCase> {};

TEST_P(FactoryParamTest, CreatesCorrectRunner) {
  auto& [name, dataset, sensor, expect_clahe, expect_imread] = GetParam();
  RunConfig config;
  config.dataset   = dataset;
  config.sensor    = sensor;
  config.data_dirs = {"/data/dummy"};
  auto runner      = createDatasetRunner(config);
  EXPECT_EQ(runner->sensor(), sensor);
  if (expect_clahe) EXPECT_TRUE(runner->useClahe());
  if (expect_imread >= 0) EXPECT_EQ(runner->imreadMode(), expect_imread);
}

INSTANTIATE_TEST_SUITE_P(
  Factory,
  FactoryParamTest,
  ::testing::Values(
    FactoryCase{"EurocMono", DatasetType::EuRoC, System::MONOCULAR, false, -1},
    FactoryCase{"EurocStereoInertial", DatasetType::EuRoC, System::IMU_STEREO, false, -1},
    FactoryCase{"TumRgbd", DatasetType::TUM, System::RGBD, false, -1},
    FactoryCase{"TumMono", DatasetType::TUM, System::MONOCULAR, false, -1},
    FactoryCase{"TumviMonoInertial", DatasetType::TumVI, System::IMU_MONOCULAR, true, cv::IMREAD_GRAYSCALE},
    FactoryCase{"TumviStereo", DatasetType::TumVI, System::STEREO, true, -1}
  ),
  [](const auto& info) { return info.param.name; }
);

TEST(FactoryTest, UnknownDatasetThrows) {
  RunConfig config;
  config.dataset = static_cast<DatasetType>(99);
  EXPECT_THROW(createDatasetRunner(config), std::invalid_argument);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// sequenceParam Tests                                                                            //

struct SequenceParamCase {
  std::string name;
  DatasetType dataset;
  std::string output_dir; // empty = don't set
  std::string expected;
};

class SequenceParamParamTest : public ::testing::TestWithParam<SequenceParamCase> {};

TEST_P(SequenceParamParamTest, ReturnsExpectedParam) {
  auto& [name, dataset, output_dir, expected] = GetParam();
  RunConfig config;
  config.dataset   = dataset;
  config.sensor    = System::MONOCULAR;
  config.data_dirs = {"/data/dummy"};
  if (!output_dir.empty()) config.output_dir = output_dir;
  auto runner = createDatasetRunner(config);
  EXPECT_EQ(runner->param(), expected);
}

INSTANTIATE_TEST_SUITE_P(
  SequenceParam,
  SequenceParamParamTest,
  ::testing::Values(
    SequenceParamCase{"EurocReturnsEmpty", DatasetType::EuRoC, "", ""},
    SequenceParamCase{"TumReturnsEmpty", DatasetType::TUM, "", ""},
    SequenceParamCase{"TumviReturnsOutputDir", DatasetType::TumVI, "/custom/output", "/custom/output"}
  ),
  [](const auto& info) { return info.param.name; }
);

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// IMU Sync Tests (synthetic data injected directly into Sequence)                                //

class TestableEuRoCRunner : public EuRoCRunner {
public:
  using EuRoCRunner::_sequences;
  using EuRoCRunner::EuRoCRunner;
};

class ImuSyncTest : public ::testing::Test {
protected:
  void SetUp() override {
    RunConfig config;
    config.dataset   = DatasetType::EuRoC;
    config.sensor    = System::IMU_MONOCULAR;
    config.data_dirs = {"/dummy"};
    _runner          = std::make_unique<TestableEuRoCRunner>(config);

    _runner->_sequences.resize(1);
    auto& sequence          = _runner->_sequences[0];
    sequence.img_timestamps = {1.0, 2.0, 3.0, 4.0, 5.0};
    sequence.imu_timestamps = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
    sequence.acc.resize(10, cv::Point3f(0.f, 0.f, 9.81f));
    sequence.gyro.resize(10, cv::Point3f(0.f, 0.f, 0.f));

    sequence.first_imu = 0;
    while (sequence.first_imu < sequence.imu_timestamps.size()
           && sequence.imu_timestamps[sequence.first_imu] <= sequence.img_timestamps[0]) {
      sequence.first_imu++;
    }
    if (sequence.first_imu > 0) sequence.first_imu--;
  }

  std::unique_ptr<TestableEuRoCRunner> _runner;
};

TEST_F(ImuSyncTest, CollectImuAtFrame0ReturnsEmpty) {
  EXPECT_TRUE(_runner->readIMU(0, 0).empty());
}

TEST_F(ImuSyncTest, CollectImuAtFrame1ReturnsCorrectSlice) {
  EXPECT_EQ(_runner->readIMU(0, 1).size(), 3u);
}

TEST_F(ImuSyncTest, CollectImuAdvancesFirstImu) {
  _runner->readIMU(0, 1);
  auto& sequence = _runner->_sequences[0];
  EXPECT_EQ(sequence.first_imu, 4);

  EXPECT_EQ(_runner->readIMU(0, 2).size(), 2u);
  EXPECT_EQ(sequence.first_imu, 6);
}

TEST_F(ImuSyncTest, CollectImuAllFramesConsumesAll) {
  size_t total = 0;
  for (int ni = 0; ni < 5; ni++) total += _runner->readIMU(0, ni).size();
  EXPECT_EQ(total, 9u);
}

TEST_F(ImuSyncTest, CollectImuBoundsCheckPreventsOverread) {
  auto& sequence     = _runner->_sequences[0];
  sequence.first_imu = 9;
  EXPECT_EQ(_runner->readIMU(0, 4).size(), 1u);
  EXPECT_EQ(sequence.first_imu, 10);
  EXPECT_TRUE(_runner->readIMU(0, 4).empty());
}

TEST_F(ImuSyncTest, FirstImuComputedCorrectly) {
  EXPECT_EQ(_runner->_sequences[0].first_imu, 1);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Non-IMU runner readIMU always returns empty                                                    //

TEST(NonImuRunnerTest, TumCollectImuReturnsEmpty) {
  RunConfig config;
  config.dataset   = DatasetType::TUM;
  config.sensor    = System::MONOCULAR;
  config.data_dirs = {"/data/tum/fr1"};
  auto runner      = createDatasetRunner(config);
  EXPECT_TRUE(runner->readIMU(0, 5).empty());
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// TUM Auto-Association Tests                                                                     //

class TumAssociationTest : public ::testing::Test {
protected:
  void SetUp() override {
    _tmp_dir = fs::temp_directory_path() / "orbslam3_test_tum_assoc";
    fs::create_directories(_tmp_dir);
    writeTumFile(_tmp_dir / "rgb.txt", "# color images\n# timestamp filename\n# ...\n",
                 {"1.000 rgb/1.000.png", "1.100 rgb/1.100.png", "1.200 rgb/1.200.png",
                  "1.300 rgb/1.300.png", "1.400 rgb/1.400.png"});
    writeTumFile(_tmp_dir / "depth.txt", "# depth images\n# timestamp filename\n# ...\n",
                 {"1.005 depth/1.005.png", "1.105 depth/1.105.png", "1.205 depth/1.205.png",
                  "1.305 depth/1.305.png", "1.405 depth/1.405.png"});
  }

  void TearDown() override { fs::remove_all(_tmp_dir); }

  static void writeTumFile(
    const fs::path& path, const std::string& header, const std::vector<std::string>& entries
  ) {
    std::ofstream f(path.string());
    f << header;
    for (const auto& e : entries) f << e << "\n";
  }

  fs::path _tmp_dir;
};

TEST_F(TumAssociationTest, RgbdAutoAssociatesCorrectly) {
  RunConfig config;
  config.dataset   = DatasetType::TUM;
  config.sensor    = System::RGBD;
  config.data_dirs = {_tmp_dir};
  auto runner      = createDatasetRunner(config);
  runner->load();
  EXPECT_EQ(runner->numSequences(), 1u);
  EXPECT_EQ(runner->numFrames(0), 5u);
}

TEST_F(TumAssociationTest, RgbdNoMatchesWithTightThreshold) {
  writeTumFile(_tmp_dir / "depth.txt", "# depth images\n# timestamp filename\n# ...\n",
               {"9999.000000 depth/far1.png", "9999.100000 depth/far2.png"});
  RunConfig config;
  config.dataset   = DatasetType::TUM;
  config.sensor    = System::RGBD;
  config.data_dirs = {_tmp_dir};
  auto runner      = createDatasetRunner(config);
  EXPECT_THROW(runner->load(), std::runtime_error);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Dataset Structure Tests (simulated directory trees)                                            //

class DatasetStructureTest : public ::testing::Test {
protected:
  void SetUp() override {
    _root = fs::temp_directory_path() / "orbslam3_structure_test";
    fs::create_directories(_root);
  }

  void TearDown() override { fs::remove_all(_root); }

  static void writeDummyImage(const fs::path& path) {
    fs::create_directories(path.parent_path());
    cv::Mat img(2, 2, CV_8UC1, cv::Scalar(128));
    cv::imwrite(path.string(), img);
  }

  /// Build runner from config in one line.
  static std::unique_ptr<DatasetRunner> makeRunner(
    DatasetType dataset, System::eSensor sensor, std::vector<fs::path> dirs,
    const std::string& output_dir = ""
  ) {
    RunConfig config;
    config.dataset   = dataset;
    config.sensor    = sensor;
    config.data_dirs = std::move(dirs);
    if (!output_dir.empty()) config.output_dir = output_dir;
    return createDatasetRunner(config);
  }

  fs::path buildTumDir(const std::string& name, int num_frames, bool rgbd) {
    auto dir = _root / name;
    fs::create_directories(dir / "rgb");
    if (rgbd) fs::create_directories(dir / "depth");

    std::ofstream rgb_file((dir / "rgb.txt").string());
    rgb_file << "# color images\n# timestamp filename\n# ...\n";

    std::ofstream depth_file;
    if (rgbd) {
      depth_file.open((dir / "depth.txt").string());
      depth_file << "# depth images\n# timestamp filename\n# ...\n";
    }

    for (int i = 0; i < num_frames; ++i) {
      double             ts = 1.0 + i * 0.1;
      std::ostringstream ts_str;
      ts_str << std::fixed << std::setprecision(4) << ts;
      std::string rgb_name = "rgb/" + ts_str.str() + ".png";
      rgb_file << ts_str.str() << " " << rgb_name << "\n";
      writeDummyImage(dir / rgb_name);

      if (rgbd) {
        double             dts = ts + 0.005;
        std::ostringstream dts_str;
        dts_str << std::fixed << std::setprecision(4) << dts;
        std::string depth_name = "depth/" + dts_str.str() + ".png";
        depth_file << dts_str.str() << " " << depth_name << "\n";
        writeDummyImage(dir / depth_name);
      }
    }
    return dir;
  }

  fs::path buildEurocDir(
    const std::string& name, int num_frames, bool stereo, bool imu, int imu_rate = 5
  ) {
    auto dir = _root / name;
    fs::create_directories(dir / "mav0" / "cam0" / "data");
    if (stereo) fs::create_directories(dir / "mav0" / "cam1" / "data");

    std::ofstream csv((dir / "mav0" / "cam0" / "data.csv").string());
    csv << "#timestamp [ns],filename\n";
    for (int i = 0; i < num_frames; ++i) {
      long long   ns       = static_cast<long long>((1.0 + i * 0.1) * 1e9);
      std::string img_name = std::to_string(ns) + ".png";
      csv << ns << "," << img_name << "\n";
      writeDummyImage(dir / "mav0" / "cam0" / "data" / img_name);
      if (stereo) writeDummyImage(dir / "mav0" / "cam1" / "data" / img_name);
    }

    if (imu) {
      fs::create_directories(dir / "mav0" / "imu0");
      std::ofstream imu_csv((dir / "mav0" / "imu0" / "data.csv").string());
      imu_csv << "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n";
      int    total_imu = num_frames * imu_rate;
      double dt        = 0.1 / imu_rate;
      for (int i = 0; i < total_imu; ++i) {
        long long ns = static_cast<long long>((1.0 + i * dt) * 1e9);
        imu_csv << ns << ",0.0,0.0,0.0,0.0,0.0,9.81\n";
      }
    }
    return dir;
  }

  fs::path _root;
};

// ── LoadAndReadFrame: parameterized across datasets ─────────────────────────── //

struct LoadReadCase {
  std::string     name;
  DatasetType     dataset;
  System::eSensor sensor;
  bool            euroc_format;
  bool            stereo;
  bool            rgbd;
  int             num_frames;
  bool            expect_right;
  bool            expect_depth;
  bool            check_grayscale;
};

class LoadReadParamTest
  : public DatasetStructureTest
  , public ::testing::WithParamInterface<LoadReadCase> {};

TEST_P(LoadReadParamTest, LoadAndReadFrame) {
  auto& p = GetParam();
  auto dir = p.euroc_format ? buildEurocDir(p.name, p.num_frames, p.stereo, false)
                            : buildTumDir(p.name, p.num_frames, p.rgbd);
  auto runner = makeRunner(p.dataset, p.sensor, {dir});
  runner->load();
  EXPECT_EQ(runner->numFrames(0), static_cast<size_t>(p.num_frames));

  cv::Mat left, right, depth;
  runner->readFrame(0, 0, 1.0f, left, right, depth);
  EXPECT_FALSE(left.empty());
  EXPECT_EQ(!right.empty(), p.expect_right);
  EXPECT_EQ(!depth.empty(), p.expect_depth);
  if (p.check_grayscale) EXPECT_EQ(left.channels(), 1);
}

INSTANTIATE_TEST_SUITE_P(
  DatasetLoadRead,
  LoadReadParamTest,
  ::testing::Values(
    //                    name               dataset          sensor              euroc stereo rgbd frames right depth gray
    LoadReadCase{"TumMono",          DatasetType::TUM,   System::MONOCULAR,    false, false, false, 4, false, false, false},
    LoadReadCase{"TumRgbd",          DatasetType::TUM,   System::RGBD,         false, false, true,  3, false, true,  false},
    LoadReadCase{"EurocMono",        DatasetType::EuRoC, System::MONOCULAR,    true,  false, false, 4, false, false, false},
    LoadReadCase{"EurocStereo",      DatasetType::EuRoC, System::STEREO,       true,  true,  false, 3, true,  false, false},
    LoadReadCase{"TumViMono",        DatasetType::TumVI, System::MONOCULAR,    true,  false, false, 4, false, false, true},
    LoadReadCase{"TumViStereo",      DatasetType::TumVI, System::STEREO,       true,  true,  false, 3, true,  false, false}
  ),
  [](const auto& info) { return info.param.name; }
);

// ── MonoInertialLoad: parameterized across EuRoC and TumVI ──────────────────── //

struct InertialLoadCase {
  std::string name;
  DatasetType dataset;
};

class InertialLoadParamTest
  : public DatasetStructureTest
  , public ::testing::WithParamInterface<InertialLoadCase> {};

TEST_P(InertialLoadParamTest, MonoInertialLoad) {
  auto& p   = GetParam();
  auto  dir = buildEurocDir(p.name, 3, false, true, 5);
  auto  runner = makeRunner(p.dataset, System::IMU_MONOCULAR, {dir});
  runner->load();
  EXPECT_EQ(runner->numFrames(0), 3u);
  EXPECT_TRUE(runner->readIMU(0, 0).empty());
  EXPECT_FALSE(runner->readIMU(0, 1).empty());
}

INSTANTIATE_TEST_SUITE_P(
  InertialLoad,
  InertialLoadParamTest,
  ::testing::Values(
    InertialLoadCase{"EurocImu", DatasetType::EuRoC},
    InertialLoadCase{"TumViImu", DatasetType::TumVI}
  ),
  [](const auto& info) { return info.param.name; }
);

// ── MultiSequence: parameterized across all three datasets ──────────────────── //

struct MultiSeqCase {
  std::string     name;
  DatasetType     dataset;
  System::eSensor sensor;
  bool            euroc_format;
};

class MultiSeqParamTest
  : public DatasetStructureTest
  , public ::testing::WithParamInterface<MultiSeqCase> {};

TEST_P(MultiSeqParamTest, MultiSequenceLoad) {
  auto& p = GetParam();
  auto dir1 = p.euroc_format ? buildEurocDir(p.name + "_s1", 4, false, false)
                             : buildTumDir(p.name + "_s1", 4, false);
  auto dir2 = p.euroc_format ? buildEurocDir(p.name + "_s2", 2, false, false)
                             : buildTumDir(p.name + "_s2", 2, false);
  auto runner = makeRunner(p.dataset, p.sensor, {dir1, dir2});
  runner->load();
  EXPECT_EQ(runner->numSequences(), 2u);
  EXPECT_EQ(runner->numFrames(0), 4u);
  EXPECT_EQ(runner->numFrames(1), 2u);
}

INSTANTIATE_TEST_SUITE_P(
  MultiSeq,
  MultiSeqParamTest,
  ::testing::Values(
    MultiSeqCase{"Tum", DatasetType::TUM, System::MONOCULAR, false},
    MultiSeqCase{"Euroc", DatasetType::EuRoC, System::MONOCULAR, true},
    MultiSeqCase{"TumVi", DatasetType::TumVI, System::MONOCULAR, true}
  ),
  [](const auto& info) { return info.param.name; }
);

// ── Empty data edge cases ───────────────────────────────────────────────────── //

TEST_F(DatasetStructureTest, TumEmptyRgbThrows) {
  auto dir = _root / "tum_empty";
  fs::create_directories(dir);
  std::ofstream((dir / "rgb.txt").string()) << "# color images\n# timestamp filename\n# ...\n";
  auto runner = makeRunner(DatasetType::TUM, System::MONOCULAR, {dir});
  EXPECT_THROW(runner->load(), std::runtime_error);
}

TEST_F(DatasetStructureTest, EurocEmptyCsvThrows) {
  auto dir = _root / "euroc_empty";
  fs::create_directories(dir / "mav0" / "cam0" / "data");
  std::ofstream((dir / "mav0" / "cam0" / "data.csv").string()) << "#timestamp [ns],filename\n";
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir});
  EXPECT_THROW(runner->load(), std::runtime_error);
}

// ── TUM-VI param test ───────────────────────────────────────────────────────── //

TEST_F(DatasetStructureTest, TumViParam) {
  auto dir    = buildEurocDir("tumvi_param", 2, false, false);
  auto runner = makeRunner(DatasetType::TumVI, System::MONOCULAR, {dir}, "/custom/output");
  EXPECT_EQ(runner->param(), "/custom/output");
}

// ── Cross-cutting Tests ─────────────────────────────────────────────────────── //

TEST_F(DatasetStructureTest, ReadFrameResizesImage) {
  auto dir    = buildEurocDir("euroc_resize", 1, false, false);
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir});
  runner->load();

  cv::Mat left, right, depth;
  runner->readFrame(0, 0, 0.5f, left, right, depth);
  EXPECT_EQ(left.rows, 1);
  EXPECT_EQ(left.cols, 1);
}

TEST_F(DatasetStructureTest, ReadFrameWithInvalidImageThrows) {
  auto dir = _root / "euroc_noimg";
  fs::create_directories(dir / "mav0" / "cam0" / "data");
  std::ofstream((dir / "mav0" / "cam0" / "data.csv").string())
    << "#timestamp [ns],filename\n1000000000,missing.png\n";
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir});
  runner->load();
  EXPECT_EQ(runner->numFrames(0), 1u);

  cv::Mat left, right, depth;
  EXPECT_THROW(runner->readFrame(0, 0, 1.0f, left, right, depth), std::runtime_error);
}

// ── Timestamp check for EuRoC/TUM ──────────────────────────────────────────── //

TEST_F(DatasetStructureTest, EurocTimestampsCorrect) {
  auto dir    = buildEurocDir("euroc_ts", 4, false, false);
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir});
  runner->load();
  EXPECT_EQ(runner->numSequences(), 1u);
  EXPECT_NEAR(runner->timestamp(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(runner->timestamp(0, 3), 1.3, 1e-3);
}

TEST_F(DatasetStructureTest, TumTimestampsCorrect) {
  auto dir    = buildTumDir("tum_ts", 4, false);
  auto runner = makeRunner(DatasetType::TUM, System::MONOCULAR, {dir});
  runner->load();
  EXPECT_NEAR(runner->timestamp(0, 0), 1.0, 1e-4);
  EXPECT_NEAR(runner->timestamp(0, 3), 1.3, 1e-4);
}

} // namespace ORB_SLAM3
