#ifndef HDMAP_RVIZ_PLUGIN_UTIL_CURRENT_REGION_H_
#define HDMAP_RVIZ_PLUGIN_UTIL_CURRENT_REGION_H_

#include <OgreColourValue.h>

#include <memory>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace hdmap_rviz_plugins {

class CurrentRegion {
 public:
  CurrentRegion(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node)
      : scene_manager_(scene_manager),
        scene_node_(scene_node),
        line_width_(0.5f),
        boundary_(std::make_shared<rviz_rendering::BillboardLine>(
            scene_manager_, scene_node_)) {
    boundary_->setNumLines(1);
    boundary_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
    boundary_->setLineWidth(line_width_);
  }

  std::string lane_id();

  void set_lane_id(const std::string& lane_id);

  std::shared_ptr<rviz_rendering::BillboardLine> mutable_boundary();

 private:
  Ogre::SceneManager* scene_manager_;

  Ogre::SceneNode* scene_node_;

  std::string lane_id_;

  float line_width_;

  std::shared_ptr<rviz_rendering::BillboardLine> boundary_;
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_UTIL_CURRENT_REGION_H_
