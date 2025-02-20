#include "libmotioncapture/vicon.h"
#include <iostream>
#include <stdio.h>  // for fprintf, stderr, NULL, etc

// VICON
#include "ViconDataStreamSDK_CPP/DataStreamClient.h"

using namespace ViconDataStreamSDK::CPP;

namespace libmotioncapture {

  class MotionCaptureViconImpl
  {
  public:
    Client client;
    std::string version;
  };

  MotionCaptureVicon::MotionCaptureVicon(
    const std::string& hostname,
    bool enableObjects,
    bool enablePointcloud,
    bool addLabeledMarkersToPointcloud)
  {
    pImpl = new MotionCaptureViconImpl;
    this->addLabeledMarkersToPointcloud = addLabeledMarkersToPointcloud;

    // Try connecting...
    while (!pImpl->client.IsConnected().Connected) {
      pImpl->client.Connect(hostname);
    }

    if (enableObjects) {
      pImpl->client.EnableSegmentData();
    }
    if (enablePointcloud) {
      pImpl->client.EnableUnlabeledMarkerData();

      if(addLabeledMarkersToPointcloud) {
        pImpl->client.EnableMarkerData();
      }
    }
    // This is the lowest latency option
    pImpl->client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Set the global up axis
    pImpl->client.SetAxisMapping(Direction::Forward,
                          Direction::Left,
                          Direction::Up); // Z-up

    // Discover the version number
    Output_GetVersion version = pImpl->client.GetVersion();
    std::stringstream sstr;
    sstr << version.Major << "." << version.Minor << "." << version.Point;
    pImpl->version = sstr.str();
  }

  MotionCaptureVicon::~MotionCaptureVicon()
  {
    delete pImpl;
  }

  const std::string& MotionCaptureVicon::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureVicon::waitForNextFrame()
  {
    while (pImpl->client.GetFrame().Result != Result::Success) {
    }
  }

  const std::map<std::string, RigidBody>& MotionCaptureVicon::rigidBodies() const
  {
    rigidBodies_.clear();
    size_t count = pImpl->client.GetSubjectCount().SubjectCount;
    for (size_t i = 0; i < count; ++i) {
      auto const name = pImpl->client.GetSubjectName(i).SubjectName;
      auto const translation = pImpl->client.GetSegmentGlobalTranslation(name, name);
      auto const quaternion = pImpl->client.GetSegmentGlobalRotationQuaternion(name, name);
      if (   translation.Result == Result::Success
          && quaternion.Result == Result::Success
          && !translation.Occluded
          && !quaternion.Occluded) {

        Eigen::Vector3f position(
          translation.Translation[0] / 1000.0,
          translation.Translation[1] / 1000.0,
          translation.Translation[2] / 1000.0);

        Eigen::Quaternionf rotation(
          quaternion.Rotation[3], // w
          quaternion.Rotation[0], // x
          quaternion.Rotation[1], // y
          quaternion.Rotation[2]  // z
          );

        rigidBodies_.emplace(name, RigidBody(name, position, rotation));
      }
    }
    return rigidBodies_;
  }

  RigidBody MotionCaptureVicon::rigidBodyByName(
    const std::string& name) const
  {
    auto const translation = pImpl->client.GetSegmentGlobalTranslation(name, name);
    auto const quaternion = pImpl->client.GetSegmentGlobalRotationQuaternion(name, name);
    if (   translation.Result == Result::Success
        && quaternion.Result == Result::Success
        && !translation.Occluded
        && !quaternion.Occluded) {

      Eigen::Vector3f position(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0);

      Eigen::Quaternionf rotation(
        quaternion.Rotation[3], // w
        quaternion.Rotation[0], // x
        quaternion.Rotation[1], // y
        quaternion.Rotation[2]  // z
        );

      return RigidBody(name, position, rotation);
    }
    throw std::runtime_error("Unknown rigid body!");
  }

  const PointCloud& MotionCaptureVicon::pointCloud() const
  {
    size_t count = pImpl->client.GetUnlabeledMarkerCount().MarkerCount;
    size_t count_labeled = pImpl->client.GetLabeledMarkerCount().MarkerCount;

    if(this->addLabeledMarkersToPointcloud) { 
      pointcloud_.resize(count + count_labeled, Eigen::NoChange);
    } else {
      pointcloud_.resize(count, Eigen::NoChange);
    }


    for(size_t i = 0; i < count; ++i) {
      Output_GetUnlabeledMarkerGlobalTranslation translation =
        pImpl->client.GetUnlabeledMarkerGlobalTranslation(i);
      pointcloud_.row(i) << 
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0;
    }

    // most likely a little bit ugly
    if(this->addLabeledMarkersToPointcloud) {
      for(size_t i = 0; i < count_labeled; ++i) {
        Output_GetLabeledMarkerGlobalTranslation translation =
          pImpl->client.GetLabeledMarkerGlobalTranslation(i);
        pointcloud_.row(i + count) << 
          translation.Translation[0] / 1000.0,
          translation.Translation[1] / 1000.0,
          translation.Translation[2] / 1000.0;
      }
    }
    return pointcloud_;
  }

  const std::vector<LatencyInfo>& MotionCaptureVicon::latency() const
  {
    latencies_.clear();
    size_t latencyCount = pImpl->client.GetLatencySampleCount().Count;
    for(size_t i = 0; i < latencyCount; ++i) {
      std::string sampleName  = pImpl->client.GetLatencySampleName(i).Name;
      double      sampleValue = pImpl->client.GetLatencySampleValue(sampleName).Value;
      latencies_.emplace_back(LatencyInfo(sampleName, sampleValue));
    }
    return latencies_;
  }

}
