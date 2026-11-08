#include "libmotioncapture/vicon.h"
#include <vector>

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
    : addLabeledMarkersToPointcloud(addLabeledMarkersToPointcloud)
  {
    pImpl = new MotionCaptureViconImpl;

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
        pImpl->client.EnableMarkerRayData();
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
    auto& client = pImpl->client;
    const auto unlabeled = client.GetUnlabeledMarkerCount();
    std::vector<Eigen::Vector3f> points;
    points.reserve(unlabeled.MarkerCount);

    for (size_t i = 0; i < unlabeled.MarkerCount; ++i) {
      const auto translation = client.GetUnlabeledMarkerGlobalTranslation(i);
      points.emplace_back(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0);
    }

    if (addLabeledMarkersToPointcloud) {
      // Camera-ray information is available through subject and marker names.
      constexpr unsigned int minimumRayContributions = 3;
      const auto subjects = client.GetSubjectCount();
      if (subjects.Result == Result::Success) {
        for (size_t i = 0; i < subjects.SubjectCount; ++i) {
          const auto subject = client.GetSubjectName(i);
          if (subject.Result != Result::Success) continue;

          const auto markers = client.GetMarkerCount(subject.SubjectName);
          if (markers.Result != Result::Success) continue;

          for (size_t j = 0; j < markers.MarkerCount; ++j) {
            const auto marker = client.GetMarkerName(subject.SubjectName, j);
            if (marker.Result != Result::Success) continue;

            const auto translation = client.GetMarkerGlobalTranslation(
              subject.SubjectName, marker.MarkerName);
            if (translation.Result != Result::Success || translation.Occluded) continue;

            const auto rays = client.GetMarkerRayContributionCount(
              subject.SubjectName, marker.MarkerName);
            if (rays.Result != Result::Success ||
                rays.RayContributionsCount < minimumRayContributions) continue;

            points.emplace_back(
              translation.Translation[0] / 1000.0,
              translation.Translation[1] / 1000.0,
              translation.Translation[2] / 1000.0);
          }
        }
      }
    }

    pointcloud_.resize(points.size(), Eigen::NoChange);
    for (size_t i = 0; i < points.size(); ++i) {
      pointcloud_.row(i) = points[i].transpose();
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
