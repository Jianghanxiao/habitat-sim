// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <string>
#include <vector>

#include "Asset.h"
#include "Attributes.h"
#include "BaseMesh.h"
#include "CollisionMeshData.h"
#include "GltfMeshData.h"
#include "MeshData.h"
#include "MeshMetaData.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
class Drawable;
}
namespace scene {
struct SceneConfiguration;
}
namespace physics {
class PhysicsManager;
class RigidObject;
}  // namespace physics

namespace assets {

typedef struct link {
  std::string link_name = "";
  std::string mesh_name = "";
  vec3f origin;

  // Store the articulations information
  std::string joint_type = "fixed";
  vec3f joint_origin;  // Relative to the link itself
  vec2f joint_limit;
  vec3f joint_axis;

  // Store the parent and child link
  struct link* parent_link = NULL;
  std::vector<struct link*> child_link;
} Link;

typedef struct joint {
  std::string parent_name = "";
  std::string child_name = "";
  std::string joint_type = "fixed";
  vec3f origin;
  vec2f limit;
  vec3f axis;
} Joint;

class URDFParser {
 public:
  URDFParser() {}
  URDFParser(const std::string& filename);
  ~URDFParser() {}

  void set(const std::string& filename);
  bool parse();

  Link* getRoot() { return root_; }

 protected:
  std::string filename_;
  Link* root_ = NULL;
};

}  // namespace assets
}  // namespace esp