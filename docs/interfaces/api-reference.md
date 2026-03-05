# API Reference

> **Namespace:** `ORB_SLAM3`
> **Headers:** `include/System.h`, `include/Common/DatasetRunner.h`, `include/Settings.h`

---

## System Class

**Header:** `include/System.h`

Main entry point. Launches Local Mapping, Loop Closing, and Viewer threads on construction. All public tracking methods are thread-safe.

### Enumerations

#### eSensor

| Value | Int | Description |
|-------|-----|-------------|
| `MONOCULAR` | 0 | Single camera |
| `STEREO` | 1 | Rectified stereo pair |
| `RGBD` | 2 | RGB-D camera |
| `IMU_MONOCULAR` | 3 | Monocular with inertial |
| `IMU_STEREO` | 4 | Stereo with inertial |
| `IMU_RGBD` | 5 | RGB-D with inertial |

#### FileType

| Value | Int | Description |
|-------|-----|-------------|
| `TEXT_FILE` | 0 | Human-readable atlas format |
| `BINARY_FILE` | 1 | Binary atlas format |

### Constructor

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `strVocFile` | `const string&` | | Path to ORB vocabulary file |
| `strSettingsFile` | `const string&` | | Path to YAML settings file |
| `sensor` | `eSensor` | | Input sensor configuration |
| `bUseViewer` | `bool` | `true` | Launch Pangolin viewer thread |
| `initFr` | `int` | `0` | Initial frame index |
| `strSequence` | `const string&` | `""` | Sequence identifier for multi-session |

### Tracking Methods

All return `Sophus::SE3f` (camera pose in world frame). Returns empty SE3 if tracking fails.

| Method | Parameters | Sensor Constraint |
|--------|-----------|-------------------|
| `TrackStereo` | imLeft, imRight, timestamp, [vImuMeas], [filename] | STEREO, IMU_STEREO |
| `TrackRGBD` | im, depthmap, timestamp, [vImuMeas], [filename] | RGBD, IMU_RGBD |
| `TrackMonocular` | im, timestamp, [vImuMeas], [filename] | MONOCULAR, IMU_MONOCULAR |

**Image format:** RGB (`CV_8UC3`) or grayscale (`CV_8U`); RGB is converted internally.
**Depth format:** Float (`CV_32F`), registered to the RGB frame.
**Stereo requirement:** Images must be rectified and synchronized.

### Trajectory Output

Call `Shutdown()` before saving. Each method writes to the given file path.

| Method | Format | Sensor Constraint |
|--------|--------|-------------------|
| `SaveTrajectoryTUM(filename)` | TUM RGB-D (seconds) | Stereo/RGBD only |
| `SaveKeyFrameTrajectoryTUM(filename)` | TUM RGB-D keyframes | All sensors |
| `SaveTrajectoryEuRoC(filename)` | EuRoC (nanoseconds) | All sensors |
| `SaveKeyFrameTrajectoryEuRoC(filename)` | EuRoC keyframes | All sensors |
| `SaveTrajectoryKITTI(filename)` | KITTI odometry | Stereo/RGBD only |

Per-map overloads of the EuRoC methods accept an additional `Map*` parameter.

### Lifecycle Control

| Method | Description |
|--------|-------------|
| `Shutdown()` | Request all threads to finish; blocks until complete |
| `isShutDown()` | Returns `true` after shutdown completes |
| `Reset()` | Clear entire atlas |
| `ResetActiveMap()` | Clear only the active map |
| `ChangeDataset()` | Notify system of sequence boundary in multi-sequence runs |
| `ActivateLocalizationMode()` | Stop mapping; tracking and relocalization only |
| `DeactivateLocalizationMode()` | Resume full SLAM (mapping + tracking) |

### State Query

| Method | Return Type | Description |
|--------|-------------|-------------|
| `GetTrackingState()` | `int` | Current tracking state (see `eTrackingState`) |
| `GetTrackedMapPoints()` | `vector<MapPoint*>` | Map points visible in the last frame |
| `GetTrackedKeyPointsUn()` | `vector<cv::KeyPoint>` | Undistorted keypoints from the last frame |
| `MapChanged()` | `bool` | True if loop closure or global BA occurred since last call |
| `GetImageScale()` | `float` | Image scale factor from settings |
| `isLost()` | `bool` | True if tracking is in LOST state |
| `isFinished()` | `bool` | True if system processing is complete |

---

## Tracking::eTrackingState Enum

**Header:** `include/Tracking.h`

Returned as `int` by `System::GetTrackingState()`.

| Value | Int | Description |
|-------|-----|-------------|
| `SYSTEM_NOT_READY` | -1 | System still initializing |
| `NO_IMAGES_YET` | 0 | No frames have been processed |
| `NOT_INITIALIZED` | 1 | Awaiting visual initialization |
| `OK` | 2 | Normal tracking |
| `RECENTLY_LOST` | 3 | Tracking lost, attempting recovery |
| `LOST` | 4 | Tracking failed; requires relocalization |
| `OK_KLT` | 5 | KLT optical-flow tracking mode |

---

## DatasetRunner Interface

**Header:** `include/Common/DatasetRunner.h`

Abstract Strategy pattern for offline dataset processing. Concrete implementations handle dataset-specific loading while frame reading and IMU collection are shared in the base class.

### DatasetType Enum

| Value | Description |
|-------|-------------|
| `EuRoC` | EuRoC MAV dataset |
| `TUM` | TUM RGB-D benchmark |
| `TumVI` | TUM Visual-Inertial dataset |

### RunConfig Struct

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `dataset` | `DatasetType` | | Dataset format identifier |
| `sensor` | `eSensor` | | Sensor configuration |
| `data_dirs` | `vector<path>` | | Paths to sequence data directories |
| `vocabulary_file` | `path` | | ORB vocabulary file path |
| `settings_file` | `path` | | YAML camera/algorithm settings path |
| `output_dir` | `path` | `"/tmp"` | Trajectory output directory |
| `use_viewer` | `bool` | `true` | Enable Pangolin viewer |

### DatasetRunner Methods

| Method | Return | Description |
|--------|--------|-------------|
| `load()` | `void` | **Pure virtual.** Load all sequence data from disk |
| `sensor()` | `eSensor` | **Pure virtual.** Sensor type for System construction |
| `numSequences()` | `size_t` | Number of loaded sequences |
| `numFrames(seq)` | `size_t` | Number of images in sequence `seq` |
| `timestamp(seq, frame)` | `double` | Camera timestamp in seconds |
| `readFrame(seq, frame, img_scale, &img_left, &img_right, &depth)` | `double` | Read and optionally resize frame images; returns timestamp |
| `readIMU(seq, frame)` | `vector<IMU::Point>` | Collect IMU samples between previous and current frame. **Mutates internal cursor; call once per frame, in order.** |
| `useClahe()` | `bool` | Whether CLAHE equalization is needed (true for TUM-VI) |
| `imreadMode()` | `int` | OpenCV imread flag (`IMREAD_GRAYSCALE` for TUM-VI, `IMREAD_UNCHANGED` otherwise) |
| `param()` | `string` | Sequence string for System constructor (non-empty for TUM-VI only) |

### Free Functions

| Function | Description |
|----------|-------------|
| `createDatasetRunner(config)` | Factory. Returns `unique_ptr<DatasetRunner>` for the configured dataset type |
| `parseArgs(argc, argv, &config)` | Parse CLI arguments into `RunConfig`. Returns `false` if `--help` requested |
| `saveTrajectories(slam, output_dir, dataset, sensor)` | Save trajectory in the dataset-appropriate format |

---

## Settings Class

**Header:** `include/Settings.h`

Parses YAML configuration files. Constructed with a config file path and sensor type integer. Holds all runtime parameters grouped by subsystem.

### CameraType Enum

| Value | Int | Description |
|-------|-----|-------------|
| `PinHole` | 0 | Standard pinhole model |
| `Rectified` | 1 | Pre-rectified stereo |
| `KannalaBrandt` | 2 | Fisheye (Kannala-Brandt) model |

### Settings Accessors

**Camera geometry:**
`cameraType()`, `camera1()`, `camera2()`, `camera1DistortionCoef()`, `camera2DistortionCoef()`, `Tlr()`, `bf()`, `b()`, `thDepth()`, `needToUndistort()`, `needToRectify()`, `needToResize()`, `M1l()`, `M2l()`, `M1r()`, `M2r()`

**Image:**
`newImSize()`, `fps()`, `rgb()`

**IMU:**
`noiseGyro()`, `noiseAcc()`, `gyroWalk()`, `accWalk()`, `imuFrequency()`, `Tbc()`, `insertKFsWhenLost()`

**ORB features:**
`nFeatures()`, `nLevels()`, `scaleFactor()`, `initThFAST()`, `minThFAST()`

**RGB-D:**
`depthMapFactor()`

**Viewer:**
`keyFrameSize()`, `keyFrameLineWidth()`, `graphLineWidth()`, `pointSize()`, `cameraSize()`, `cameraLineWidth()`, `viewPointX()`, `viewPointY()`, `viewPointZ()`, `viewPointF()`, `imageViewerScale()`

**Persistence:**
`atlasLoadFile()`, `atlasSaveFile()`, `thFarPoints()`

**String representation:**
`Str()` -- returns a formatted summary of all loaded settings.
