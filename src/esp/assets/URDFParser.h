// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <string>
#include <vector>

#include "Asset.h"

namespace esp {

namespace assets {

typedef struct link {
  std::string link_name = "";
  std::string mesh_name = "";
  vec3f origin = vec3f(0, 0, 0);

  // Store the articulations information
  std::string joint_type = "fixed";
  vec3f joint_origin = vec3f(0, 0, 0);  // Relative to the link itself
  vec2f joint_limit = vec2f(-1, -1);
  vec3f joint_axis = vec3f(0, 0, 0);

  // Store the parent and child link
  struct link* parent_link = NULL;
  std::vector<struct link*> child_link;
} Link;

typedef struct joint {
  std::string parent_name = "";
  std::string child_name = "";
  std::string joint_type = "fixed";
  vec3f origin = vec3f(0, 0, 0);
  vec2f limit = vec2f(-1, -1);
  vec3f axis = vec3f(0, 0, 0);
} Joint;

class URDFParser {
 public:
  URDFParser() {}
  URDFParser(const std::string& filename);
  ~URDFParser();

  void set(const std::string& filename);
  bool parse();

  const Link& getRoot() const { return *root_; }

 protected:
  std::string filename_;
  Link* root_ = NULL;
  std::vector<Link*> link_vec_;
  std::vector<Joint*> joint_vec_;
};

}  // namespace assets
}  // namespace esp