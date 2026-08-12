#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#ifndef Q_MOC_RUN
#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRenderOperation.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>
#endif

#include <pluginlib/class_list_macros.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/status_property.hpp"

namespace lunar_rover_mapping
{

class RainbowOccupancyGridDisplay
  : public rviz_common::MessageFilterDisplay<nav_msgs::msg::OccupancyGrid>
{
  Q_OBJECT

public:
  RainbowOccupancyGridDisplay()
  {
    alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 0.88f,
      "Opacity for the generated 2D terrain pixels.",
      this, SLOT(updateStyle()));
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    min_value_property_ = new rviz_common::properties::IntProperty(
      "Min Value", 5,
      "OccupancyGrid value mapped to the low end of the terrain palette.",
      this, SLOT(updateStyle()), this, 0, 100);
    max_value_property_ = new rviz_common::properties::IntProperty(
      "Max Value", 55,
      "OccupancyGrid value mapped to the high end of the terrain palette.",
      this, SLOT(updateStyle()), this, 0, 100);
    wall_threshold_property_ = new rviz_common::properties::IntProperty(
      "Wall Threshold", 100,
      "Cells at or above this value are drawn with Wall Color.",
      this, SLOT(updateStyle()), this, 0, 100);
    wall_color_property_ = new rviz_common::properties::ColorProperty(
      "Wall Color", QColor(255, 25, 20),
      "Color for wall cells.",
      this, SLOT(updateStyle()), this);
    unknown_cells_property_ = new rviz_common::properties::BoolProperty(
      "Draw Unknown Cells", false,
      "Draw unknown cells as transparent dark pixels instead of skipping them.",
      this, SLOT(updateStyle()));
    free_cells_property_ = new rviz_common::properties::BoolProperty(
      "Draw Free Cells", false,
      "Draw free cells as low-height terrain pixels.",
      this, SLOT(updateStyle()));
    z_offset_property_ = new rviz_common::properties::FloatProperty(
      "Z Offset", 0.025f,
      "Small vertical offset to prevent z-fighting with world geometry.",
      this, SLOT(updateStyle()));
  }

  ~RainbowOccupancyGridDisplay() override
  {
    clear();
    if (!material_name_.empty()) {
      Ogre::MaterialManager::getSingleton().remove(material_name_);
    }
  }

  void onInitialize() override
  {
    MFDClass::onInitialize();
    material_name_ = "lunar_rover_mapping/RainbowOccupancyGrid/" +
      std::to_string(reinterpret_cast<std::uintptr_t>(this));
    auto material = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    auto * pass = material->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthCheckEnabled(false);
    pass->setDepthWriteEnabled(false);
    material->setReceiveShadows(false);
  }

  void reset() override
  {
    MFDClass::reset();
    clear();
  }

protected:
  void onEnable() override
  {
    MFDClass::onEnable();
    if (manual_object_) {
      manual_object_->setVisible(true);
    }
  }

  void onDisable() override
  {
    MFDClass::onDisable();
    if (manual_object_) {
      manual_object_->setVisible(false);
    }
  }

  void processMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) override
  {
    if (!msg || msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0f) {
      setStatus(
        rviz_common::properties::StatusProperty::Error,
        "Map", "Received an empty or invalid OccupancyGrid.");
      clear();
      return;
    }

    geometry_msgs::msg::Pose origin = msg->info.origin;
    origin.position.z += z_offset_property_->getFloat();
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->transform(
        msg->header.frame_id, msg->header.stamp, origin, position, orientation))
    {
      setMissingTransformToFixedFrame(msg->header.frame_id);
      return;
    }

    if (!manual_object_) {
      manual_object_ = scene_manager_->createManualObject();
      manual_object_->setDynamic(true);
      scene_node_->attachObject(manual_object_);
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    rebuildGeometry(*msg);
    setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "OK");
    context_->queueRender();
  }

private Q_SLOTS:
  void updateStyle()
  {
    if (last_msg_) {
      processMessage(last_msg_);
    }
  }

private:
  void clear()
  {
    last_msg_.reset();
    if (manual_object_) {
      manual_object_->clear();
    }
  }

  static Ogre::ColourValue turboColor(double value, double alpha)
  {
    const double x = std::clamp(value, 0.0, 1.0);
    const double x2 = x * x;
    const double x3 = x2 * x;
    const double x4 = x3 * x;
    const double x5 = x4 * x;
    const auto clamp_color = [](double channel) {
        return static_cast<float>(std::clamp(channel, 0.0, 1.0));
      };

    const double r = 0.13572138 + 4.61539260 * x - 42.66032258 * x2 +
      132.13108234 * x3 - 152.94239396 * x4 + 59.28637943 * x5;
    const double g = 0.09140261 + 2.19418839 * x + 4.84296658 * x2 -
      14.18503333 * x3 + 4.27729857 * x4 + 2.82956604 * x5;
    const double b = 0.10667330 + 12.64194608 * x - 60.58204836 * x2 +
      110.36276771 * x3 - 89.90310912 * x4 + 27.34824973 * x5;
    return Ogre::ColourValue(
      clamp_color(r), clamp_color(g), clamp_color(b),
      static_cast<float>(std::clamp(alpha, 0.0, 1.0)));
  }

  Ogre::ColourValue colorForValue(int value) const
  {
    const double alpha = alpha_property_->getFloat();
    if (value < 0) {
      return Ogre::ColourValue(0.02f, 0.02f, 0.02f, static_cast<float>(0.18 * alpha));
    }
    if (value >= wall_threshold_property_->getInt()) {
      QColor wall_color = wall_color_property_->getColor();
      return Ogre::ColourValue(
        wall_color.redF(), wall_color.greenF(), wall_color.blueF(),
        static_cast<float>(alpha));
    }

    const int min_value = min_value_property_->getInt();
    const int max_value = std::max(min_value + 1, max_value_property_->getInt());
    const double ratio = static_cast<double>(value - min_value) /
      static_cast<double>(max_value - min_value);
    return turboColor(ratio, alpha);
  }

  bool shouldDrawValue(int value) const
  {
    if (value < 0) {
      return unknown_cells_property_->getBool();
    }
    if (value == 0 && !free_cells_property_->getBool()) {
      return false;
    }
    return true;
  }

  void addCell(
    float x, float y, float resolution, const Ogre::ColourValue & color,
    std::uint32_t & vertex_index)
  {
    manual_object_->position(x, y, 0.0f);
    manual_object_->colour(color);
    manual_object_->position(x + resolution, y, 0.0f);
    manual_object_->colour(color);
    manual_object_->position(x + resolution, y + resolution, 0.0f);
    manual_object_->colour(color);
    manual_object_->position(x, y + resolution, 0.0f);
    manual_object_->colour(color);

    manual_object_->triangle(vertex_index, vertex_index + 1, vertex_index + 2);
    manual_object_->triangle(vertex_index, vertex_index + 2, vertex_index + 3);
    vertex_index += 4;
  }

  void rebuildGeometry(const nav_msgs::msg::OccupancyGrid & msg)
  {
    last_msg_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(msg);
    manual_object_->clear();

    const auto cell_count = static_cast<std::size_t>(msg.info.width) *
      static_cast<std::size_t>(msg.info.height);
    const auto data_count = std::min(cell_count, msg.data.size());
    std::size_t drawn = 0;
    for (std::size_t index = 0; index < data_count; ++index) {
      if (shouldDrawValue(static_cast<int>(msg.data[index]))) {
        ++drawn;
      }
    }

    if (drawn == 0) {
      setStatus(rviz_common::properties::StatusProperty::Warn, "Map", "No drawable cells.");
      return;
    }

    manual_object_->estimateVertexCount(drawn * 4);
    manual_object_->estimateIndexCount(drawn * 6);
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    const float resolution = msg.info.resolution;
    std::uint32_t vertex_index = 0;
    for (std::size_t index = 0; index < data_count; ++index) {
      const int value = static_cast<int>(msg.data[index]);
      if (!shouldDrawValue(value)) {
        continue;
      }
      const auto row = static_cast<std::uint32_t>(index / msg.info.width);
      const auto column = static_cast<std::uint32_t>(index - row * msg.info.width);
      const auto color = colorForValue(value);
      addCell(column * resolution, row * resolution, resolution, color, vertex_index);
    }

    manual_object_->end();
  }

  Ogre::ManualObject * manual_object_{nullptr};
  std::string material_name_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_msg_;

  rviz_common::properties::FloatProperty * alpha_property_{nullptr};
  rviz_common::properties::IntProperty * min_value_property_{nullptr};
  rviz_common::properties::IntProperty * max_value_property_{nullptr};
  rviz_common::properties::IntProperty * wall_threshold_property_{nullptr};
  rviz_common::properties::ColorProperty * wall_color_property_{nullptr};
  rviz_common::properties::BoolProperty * unknown_cells_property_{nullptr};
  rviz_common::properties::BoolProperty * free_cells_property_{nullptr};
  rviz_common::properties::FloatProperty * z_offset_property_{nullptr};
};

}  // namespace lunar_rover_mapping

PLUGINLIB_EXPORT_CLASS(lunar_rover_mapping::RainbowOccupancyGridDisplay, rviz_common::Display)

#include "rainbow_occupancy_grid_display.moc"
