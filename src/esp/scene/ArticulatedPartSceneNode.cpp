#include "ArticulatedPartSceneNode.h"
#include <iostream>

namespace esp {
namespace scene {

SceneNode& ArticulatedPartSceneNode::createArticulatedChild() {
  // will set the parent to *this
  SceneNode* node = new ArticulatedPartSceneNode(*this);
  node->setId(this->getId());
  return *node;
}

}  // namespace scene
}  // namespace esp