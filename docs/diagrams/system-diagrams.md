# ORB-SLAM3 System Diagrams

## 1. Component Diagram -- Threading Architecture

The system runs four concurrent threads sharing data through the Atlas multi-map container. Tracking runs on the caller thread; the other three are spawned at startup.

```mermaid
flowchart TB
    subgraph Input
        IMG["Images (Mono/Stereo/RGBD)"]
        IMU["IMU Measurements"]
    end

    subgraph System
        direction TB

        subgraph TrackingThread["Tracking Thread (caller thread)"]
            TRK["Tracking\n- ORB extraction\n- Pose estimation\n- Local map tracking\n- KeyFrame decision"]
        end

        subgraph LMThread["LocalMapping Thread"]
            LM["LocalMapping\n- Process new KeyFrames\n- Triangulate MapPoints\n- Local BA (g2o)\n- KeyFrame culling"]
        end

        subgraph LCThread["LoopClosing Thread"]
            LC["LoopClosing\n- Loop detection (DBoW2)\n- Sim3 computation\n- Loop correction\n- Global BA"]
        end

        subgraph ViewerThread["Viewer Thread"]
            VW["Viewer (Pangolin)"]
            FD["FrameDrawer"]
            MD["MapDrawer"]
        end

        subgraph DataLayer["Shared Data"]
            ATL["Atlas"]
            subgraph Maps["Maps (1..N)"]
                M1["Map"]
                KF["KeyFrames"]
                MP["MapPoints"]
            end
        end
    end

    IMG --> TRK
    IMU --> TRK
    TRK -- "new KeyFrame\n(queue)" --> LM
    LM -- "KeyFrame\n(queue)" --> LC
    TRK -- "read/write\npose + points" --> ATL
    LM -- "read/write\npoints + BA" --> ATL
    LC -- "read/write\nloop correction" --> ATL
    ATL --- Maps
    M1 --- KF
    M1 --- MP
    TRK --> FD
    ATL --> MD
    FD --> VW
    MD --> VW
```

## 2. Class Diagram -- Core Data Model

Core ownership and inheritance relationships. System owns all major components; Atlas manages multiple Maps, each containing sets of KeyFrames and MapPoints.

```mermaid
classDiagram
    class System {
        +eSensor mSensor
        +TrackStereo()
        +TrackRGBD()
        +TrackMonocular()
        +Shutdown()
        +SaveAtlas()
    }
    System *-- Tracking : mpTracker
    System *-- LocalMapping : mpLocalMapper
    System *-- LoopClosing : mpLoopCloser
    System *-- Viewer : mpViewer
    System *-- Atlas : mpAtlas
    System *-- FrameDrawer : mpFrameDrawer
    System *-- MapDrawer : mpMapDrawer

    class Atlas {
        -set~Map*~ mspMaps
        -Map* mpCurrentMap
        -vector~GeometricCamera*~ mvpCameras
        +CreateNewMap()
        +GetCurrentMap()
        +GetAllMaps()
    }
    Atlas "1" *-- "1..*" Map

    class Map {
        -set~KeyFrame*~ mspKeyFrames
        -set~MapPoint*~ mspMapPoints
        +GetAllKeyFrames()
        +GetAllMapPoints()
    }
    Map "1" o-- "*" KeyFrame
    Map "1" o-- "*" MapPoint

    class KeyFrame {
        +Sophus::SE3f mTcw
        +vector~cv::KeyPoint~ mvKeys
        +vector~cv::KeyPoint~ mvKeysUn
        +DBoW2::BowVector mBowVec
        +DBoW2::FeatureVector mFeatVec
        -KeyFrame* mpParent
        -set~KeyFrame*~ mspChildrens
        +GetPose()
        +GetCovisiblesByWeight()
    }
    KeyFrame "1" o-- "*" MapPoint : observes
    KeyFrame --> KeyFrame : covisibility graph\nspanning tree

    class MapPoint {
        -Eigen::Vector3f mWorldPos
        -map~KeyFrame*, tuple~ mObservations
        -cv::Mat mDescriptor
        -Eigen::Vector3f mNormalVector
        +GetWorldPos()
        +GetObservations()
    }

    class Frame {
        +Sophus::SE3f mTcw
        +vector~cv::KeyPoint~ mvKeys
        +vector~cv::KeyPoint~ mvKeysUn
        +DBoW2::BowVector mBowVec
        +ExtractORB()
    }

    class GeometricCamera {
        <<abstract>>
        +project()*
        +unproject()*
        +triangulateTwoPoints()*
    }
    class Pinhole {
        +project()
        +unproject()
    }
    class KannalaBrandt8 {
        +project()
        +unproject()
    }
    GeometricCamera <|-- Pinhole
    GeometricCamera <|-- KannalaBrandt8

    class DatasetRunner {
        <<abstract>>
        +load()*
        +sensor()*
        +run()
    }
    class EuRoCRunner
    class TumRunner
    class TumViRunner
    DatasetRunner <|-- EuRoCRunner
    DatasetRunner <|-- TumRunner
    DatasetRunner <|-- TumViRunner
    DatasetRunner --> System : constructs and feeds
```

## 3. Sequence Diagram -- Frame Processing Pipeline

End-to-end flow when a new stereo frame arrives, from image input through local mapping and loop closing.

```mermaid
sequenceDiagram
    participant App as Application
    participant Sys as System
    participant Trk as Tracking
    participant Frm as Frame
    participant LM as LocalMapping
    participant LC as LoopClosing

    App->>Sys: TrackStereo(imLeft, imRight, timestamp)
    Sys->>Trk: GrabImageStereo(imLeft, imRight, timestamp)
    Trk->>Frm: new Frame(ORB extraction, stereo matching)
    Frm-->>Trk: Frame with keypoints + descriptors

    alt state == NOT_INITIALIZED
        Trk->>Trk: StereoInitialization()
        Trk->>Trk: Create initial Map, KeyFrame, MapPoints
        Trk-->>Trk: state = OK
    else state == OK
        Trk->>Trk: TrackWithMotionModel() or TrackReferenceKeyFrame()
        Trk->>Trk: TrackLocalMap()
        Note right of Trk: Project local MapPoints,<br/>optimize pose via g2o

        Trk->>Trk: NeedNewKeyFrame()?
        alt new KeyFrame needed
            Trk->>Trk: CreateNewKeyFrame()
            Trk->>LM: InsertKeyFrame(KF)

            LM->>LM: ProcessNewKeyFrame()
            LM->>LM: MapPointCulling()
            LM->>LM: CreateNewMapPoints() (triangulation)
            LM->>LM: SearchInNeighbors() (fuse duplicates)
            LM->>LM: LocalBundleAdjustment()
            LM->>LM: KeyFrameCulling()
            LM->>LC: InsertKeyFrame(KF)

            LC->>LC: DetectLoop() via DBoW2
            alt loop detected
                LC->>LC: ComputeSim3()
                LC->>LC: CorrectLoop()
                LC->>LC: RunGlobalBundleAdjustment()
            end
        end
    end

    Trk-->>Sys: Sophus::SE3f (camera pose)
    Sys-->>App: Sophus::SE3f (camera pose)
```

## 4. State Diagram -- Tracking States

Tracking thread state machine defined in `Tracking::eTrackingState`. Values: SYSTEM_NOT_READY (-1), NO_IMAGES_YET (0), NOT_INITIALIZED (1), OK (2), RECENTLY_LOST (3), LOST (4), OK_KLT (5).

```mermaid
stateDiagram-v2
    [*] --> SYSTEM_NOT_READY : constructing

    SYSTEM_NOT_READY --> NO_IMAGES_YET : system ready

    NO_IMAGES_YET --> NOT_INITIALIZED : first image received

    NOT_INITIALIZED --> OK : initialization success
    NOT_INITIALIZED --> NOT_INITIALIZED : initialization fails,\nretry next frame

    OK --> RECENTLY_LOST : tracking fails\n(few inliers)
    OK --> OK_KLT : KLT fallback active\n(visual-inertial)

    OK_KLT --> OK : full tracking recovered
    OK_KLT --> RECENTLY_LOST : KLT tracking fails

    RECENTLY_LOST --> OK : relocalization success
    RECENTLY_LOST --> LOST : timeout exceeded\n(5s default)

    LOST --> NOT_INITIALIZED : reset / create new map
```
