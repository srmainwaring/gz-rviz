// Copyright (c) 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gz/rviz/rviz.hpp"

#include <gz/rviz/plugins/AxesDisplay.hpp>
#include <gz/rviz/plugins/GlobalOptions.hpp>
#include <gz/rviz/plugins/GPSDisplay.hpp>
#include <gz/rviz/plugins/ImageDisplay.hpp>
#include <gz/rviz/plugins/LaserScanDisplay.hpp>
#include <gz/rviz/plugins/MarkerArrayDisplay.hpp>
#include <gz/rviz/plugins/MarkerDisplay.hpp>
#include <gz/rviz/plugins/MarkerManager.hpp>
#include <gz/rviz/plugins/PathDisplay.hpp>
#include <gz/rviz/plugins/PointStampedDisplay.hpp>
#include <gz/rviz/plugins/PolygonDisplay.hpp>
#include <gz/rviz/plugins/PoseArrayDisplay.hpp>
#include <gz/rviz/plugins/PoseDisplay.hpp>
#include <gz/rviz/plugins/RobotModelDisplay.hpp>
#include <gz/rviz/plugins/TFDisplay.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gz
{
namespace rviz
{
////////////////////////////////////////////////////////////////////////////////
TopicModel::TopicModel(QObject * _parent)
: QStandardItemModel(_parent) {}

////////////////////////////////////////////////////////////////////////////////
void TopicModel::addTopic(const std::string & _name, const std::string & _msgType)
{
  QStandardItem * entry = new QStandardItem();
  entry->setData(QString::fromStdString(_name), NameRole);
  entry->setData(QString::fromStdString(_msgType), TypeRole);
  appendRow(entry);
}

////////////////////////////////////////////////////////////////////////////////
QVariant TopicModel::data(const QModelIndex & _index, int _role) const
{
  QStandardItem * myItem = itemFromIndex(_index);

  if (_role == NameRole) {
    return myItem->data(NameRole);
  }

  if (_role == TypeRole) {
    return myItem->data(TypeRole);
  }

  return QVariant();
}

////////////////////////////////////////////////////////////////////////////////
QHash<int, QByteArray> TopicModel::roleNames() const
{
  QHash<int, QByteArray> roles;
  roles[NameRole] = "topic";
  roles[TypeRole] = "msgType";
  return roles;
}

////////////////////////////////////////////////////////////////////////////////
RViz::RViz()
{
  this->topicModel = new TopicModel();

  this->supportedDisplays = {
    "geometry_msgs/msg/PointStamped",
    "geometry_msgs/msg/PolygonStamped",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseArray",
    "nav_msgs/msg/Path",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/LaserScan",
    "sensor_msgs/msg/NavSatFix",
    "visualization_msgs/msg/Marker",
    "visualization_msgs/msg/MarkerArray"
  };
}

////////////////////////////////////////////////////////////////////////////////
TopicModel * RViz::getTopicModel() const
{
  return this->topicModel;
}

////////////////////////////////////////////////////////////////////////////////
void RViz::refreshTopicList() const
{
  this->topicModel->removeRows(0, this->topicModel->rowCount());
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (std::find(
          supportedDisplays.begin(), supportedDisplays.end(),
          topicType) != this->supportedDisplays.end())
      {
        this->topicModel->addTopic(topic.first, topicType);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// void RViz::addGrid3D() const
// {
//   gz::gui::App()->LoadPlugin("Grid3D");
// }

////////////////////////////////////////////////////////////////////////////////
void RViz::addTFDisplay() const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("TFDisplay")) {
    auto tfDisplayPlugins =
      gz::gui::App()->findChildren<gz::rviz::plugins::TFDisplay *>();
    int pluginCount = tfDisplayPlugins.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    tfDisplayPlugins[pluginCount]->initialize(this->node);
    tfDisplayPlugins[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      tfDisplayPlugins[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addLaserScanDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("LaserScanDisplay")) {
    auto laserScanPlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::LaserScanDisplay *>();
    int pluginCount = laserScanPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    laserScanPlugin[pluginCount]->initialize(this->node);
    laserScanPlugin[pluginCount]->setTopic(_topic.toStdString());
    laserScanPlugin[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      laserScanPlugin[pluginCount]);
  }
}
////////////////////////////////////////////////////////////////////////////////
void RViz::addGPSDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("GPSDisplay")) {
    auto gpsDisplay =
      gz::gui::App()->findChildren<gz::rviz::plugins::GPSDisplay *>();
    int pluginCount = gpsDisplay.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    gpsDisplay[pluginCount]->initialize(this->node);
    gpsDisplay[pluginCount]->setTopic(_topic.toStdString());
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addMarkerDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("MarkerDisplay")) {
    auto markerDisplay =
      gz::gui::App()->findChildren<gz::rviz::plugins::MarkerDisplay *>();
    int pluginCount = markerDisplay.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    markerDisplay[pluginCount]->initialize(this->node);
    markerDisplay[pluginCount]->setTopic(_topic.toStdString());
    markerDisplay[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      markerDisplay[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addMarkerArrayDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("MarkerArrayDisplay")) {
    auto markerArrayDisplay =
      gz::gui::App()->findChildren<gz::rviz::plugins::MarkerArrayDisplay *>();
    int pluginCount = markerArrayDisplay.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    markerArrayDisplay[pluginCount]->initialize(this->node);
    markerArrayDisplay[pluginCount]->setTopic(_topic.toStdString());
    markerArrayDisplay[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      markerArrayDisplay[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPointStampedDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("PointStampedDisplay")) {
    auto pointStampedPlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::PointStampedDisplay *>();
    int pluginCount = pointStampedPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    pointStampedPlugin[pluginCount]->initialize(this->node);
    pointStampedPlugin[pluginCount]->setTopic(_topic.toStdString());
    pointStampedPlugin[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      pointStampedPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPolygonDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("PolygonDisplay")) {
    auto polygonPlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::PolygonDisplay *>();
    int pluginCount = polygonPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    polygonPlugin[pluginCount]->initialize(this->node);
    polygonPlugin[pluginCount]->setTopic(_topic.toStdString());
    polygonPlugin[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      polygonPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPoseDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("PoseDisplay")) {
    auto posePlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::PoseDisplay *>();
    int pluginCount = posePlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    posePlugin[pluginCount]->initialize(this->node);
    posePlugin[pluginCount]->setTopic(_topic.toStdString());
    posePlugin[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      posePlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPoseArrayDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("PoseArrayDisplay")) {
    auto poseArrayPlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::PoseArrayDisplay *>();
    int pluginCount = poseArrayPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    poseArrayPlugin[pluginCount]->initialize(this->node);
    poseArrayPlugin[pluginCount]->setTopic(_topic.toStdString());
    poseArrayPlugin[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      poseArrayPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPathDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("PathDisplay")) {
    auto pathPlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::PathDisplay *>();
    int pluginCount = pathPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    pathPlugin[pluginCount]->initialize(this->node);
    pathPlugin[pluginCount]->setTopic(_topic.toStdString());
    pathPlugin[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      pathPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addRobotModelDisplay() const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("RobotModelDisplay")) {
    auto robotModelPlugin =
      gz::gui::App()->findChildren<DisplayPlugin<std_msgs::msg::String> *>();
    int pluginCount = robotModelPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    robotModelPlugin[pluginCount]->initialize(this->node);
    robotModelPlugin[pluginCount]->setFrameManager(this->frameManager);
    robotModelPlugin[pluginCount]->setTopic("/robot_description");
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      robotModelPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addImageDisplay(const QString & _topic) const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("ImageDisplay")) {
    auto imageDisplayPlugin =
      gz::gui::App()->findChildren<gz::rviz::plugins::ImageDisplay *>();
    int pluginCount = imageDisplayPlugin.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    imageDisplayPlugin[pluginCount]->initialize(this->node);
    imageDisplayPlugin[pluginCount]->setTopic(_topic.toStdString());
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addAxesDisplay() const
{
  // Load plugin
  if (gz::gui::App()->LoadPlugin("AxesDisplay")) {
    auto axes_plugins =
      gz::gui::App()->findChildren<gz::rviz::plugins::AxesDisplay *>();
    int pluginCount = axes_plugins.size() - 1;
    if (pluginCount < 0)
      return;

    // Set frame manager and install event filter for recently added plugin
    axes_plugins[pluginCount]->setFrameManager(this->frameManager);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(
      axes_plugins[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::init_ros()
{
  this->node = std::make_shared<rclcpp::Node>("gz_rviz");
  this->frameManager = std::make_shared<common::FrameManager>(this->node);
  this->frameManager->setFixedFrame("world");
  return;

  // Load Global Options plugin
  if (gz::gui::App()->LoadPlugin("GlobalOptions")) {
    auto globalOptionsPlugin =
      gz::gui::App()->
          findChild<gz::rviz::plugins::GlobalOptions *>();

    if (globalOptionsPlugin)
    {
      // Set frame manager and install
      globalOptionsPlugin->setFrameManager(this->frameManager);

      // Install event filter
      auto mainWindow =
          gz::gui::App()->
              findChild<gz::gui::MainWindow *>();
      if (mainWindow)
      {
        mainWindow->installEventFilter(globalOptionsPlugin);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
rclcpp::Node::SharedPtr RViz::get_node()
{
  return this->node;
}

}  // namespace rviz
}  // namespace gz
