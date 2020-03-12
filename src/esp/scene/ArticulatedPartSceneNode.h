// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "SceneNode.h"

namespace esp {
namespace scene {

class ArticulatedPartSceneNode : public SceneNode {
 public:
  ArticulatedPartSceneNode() = delete;
  ArticulatedPartSceneNode(SceneNode& parent) : SceneNode(parent) {}

  ArticulatedPartSceneNode& createArticulatedChild();

  // Articulation information for URDF
  void setLinkName(std::string link_name) { link_name_ = link_name; };
  std::string getLinkName() { return link_name_; };

  void setJointType(std::string joint_type) {
    joint_type_ = joint_type;
  };
  std::string getJointType() { return joint_type_; };

  void setJointOrigin(vec3f joint_origin) {
    joint_origin_ = joint_origin;
  };
  vec3f getJointOrigin() { return joint_origin_; };

  void setJointLimit(vec2f joint_limit) {
    joint_limit_ = joint_limit;
  };
  vec2f getJointLimit() { return joint_limit_; };

  void setJointAxis(vec3f joint_axis) {
    joint_axis_ = joint_axis;
  };
  vec3f getJointAxis() { return joint_axis_; };

  void setCurrentValue(double current_value) {
    current_value_ = current_value;
  };
  double getCurrentValue() { return current_value_; };

 protected:
  // Articulations information for URDF interaction
  std::string link_name_ = "";
  std::string joint_type_ = "fixed";

  vec3f joint_origin_ = vec3f(0, 0, 0);  // Relative to the link itself
  vec2f joint_limit_ = vec2f(-1, -1);
  vec3f joint_axis_ = vec3f(0, 0, 0);

  double current_value_ = 0;
};

}  // namespace scene
}  // namespace esp