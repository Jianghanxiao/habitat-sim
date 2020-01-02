#include "ArticulatedPartSceneNode.h"
#include <iostream>

namespace esp {
namespace scene {

ArticulatedPartSceneNode& ArticulatedPartSceneNode::createArticulatedChild() {
  // will set the parent to *this
  ArticulatedPartSceneNode* node = new ArticulatedPartSceneNode(*this);
  node->setId(this->getId());
  return *node;
}

}  // namespace scene
}  // namespace esp