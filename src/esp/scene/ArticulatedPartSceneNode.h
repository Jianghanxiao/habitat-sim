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

  SceneNode& createArticulatedChild() override;

  //! Articulation information for URDF
  void setLinkName(std::string link_name) override { link_name_ = link_name; };
  std::string getLinkName() override { return link_name_; };

  void setJointType(std::string joint_type) override {
    joint_type_ = joint_type;
  };
  std::string getJointType() override { return joint_type_; };

  void setJointOrigin(vec3f joint_origin) override {
    joint_origin_ = joint_origin;
  };
  vec3f getJointOrigin() { return joint_origin_; };

  void setJointLimit(vec2f joint_limit) override {
    joint_limit_ = joint_limit;
  };
  vec2f getJointLimit() override { return joint_limit_; };

  void setJointAxis(vec3f joint_axis) override {
    joint_axis_ = joint_axis;
  };
  vec3f getJointAxis() override { return joint_axis_; };

  void setCurrentValue(double current_value) override {
    current_value_ = current_value;
  };
  double getCurrentValue() override { return current_value_; };

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