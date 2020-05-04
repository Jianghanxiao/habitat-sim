#include "URDFParser.h"
#include <iostream>

namespace esp {
namespace assets {

URDFParser::URDFParser(const std::string& filename) {
  filename_ = filename;
  root_ = NULL;
}

void URDFParser::set(const std::string& filename) {
  filename_ = filename;
  root_ = NULL;
  link_vec_.clear();
  joint_vec_.clear();
}

// Parse the URDF
bool URDFParser::parse() {
  if (root_ != NULL) {
    std::cout << "Some error in URDFParser!\n";
    return false;
  }

  // load the URDF
  const std::string& filename = filename_;
  if (freopen(filename.c_str(), "r", stdin) == NULL) {
    LOG(ERROR) << "Cannot open URDF file\n";
    std::exit(1);
  }

  int base_path_index = -1;
  for (int i = filename.length() - 1; i >= 0; --i) {
    if(filename[i] == '/') {
      base_path_index = i;
      break;
    }
  }

  std::string base_path = filename.substr(0, base_path_index + 1);

  std::string line;
  int index = -1;
  bool is_link = false, is_visual = false;
  bool is_joint = false;
  std::map<std::string, Link*> name_link_map;

  while (std::getline(std::cin, line)) {
    if (line != "") {
      // std::cout << line << "\n";
      // Judge whether this line is used to define the link
      index = line.find("<link");
      if (index != -1) {
        is_link = true;
        int name_index1 = line.find("\"");
        int name_index2 = line.find("\"", name_index1 + 1);
        link_vec_.push_back(std::make_unique<Link>());
        link_vec_[link_vec_.size() - 1]->link_name =
            line.substr(name_index1 + 1, name_index2 - name_index1 - 1);
        name_link_map[link_vec_[link_vec_.size() - 1]->link_name] =
            link_vec_[link_vec_.size() - 1].get();
      }

      // Judge whther this line is the visual attribute
      index = line.find("<visual");
      if (index != -1) {
        is_visual = true;
      }

      index = line.find("/visual>");
      if (index != -1) {
        is_visual = false;
      }

      // Get the mesh filename
      index = line.find("<mesh");
      if (index != -1 && is_link == true && is_visual == true) {
        int name_index1 = line.find("\"");
        int name_index2 = line.find("\"", name_index1 + 1);
        link_vec_[link_vec_.size() - 1]->mesh_name = base_path + 
            line.substr(name_index1 + 1, name_index2 - name_index1 - 1);
        // std::cout << name_link_map[link_vec_[link_vec_.size() -
        // 1]->link_name]
        // << "; " << link_vec_[link_vec_.size() - 1] << "\n";
      }

      index = line.find("/link>");
      if (index != -1) {
        is_link = false;
      }

      // Judge whether this line is to define a joint
      index = line.find("<joint");
      if (index != -1) {
        is_joint = true;
        joint_vec_.push_back(std::make_unique<Joint>());
      }

      // Get the joint type
      index = line.find("type");
      if (index != -1 && is_joint == true) {
        // First two index for joint name
        int name_index1 = line.find("\"");
        int name_index2 = line.find("\"", name_index1 + 1);
        // Index 3 and 4 for joint type
        int name_index3 = line.find("\"", name_index2 + 1);
        int name_index4 = line.find("\"", name_index3 + 1);
        joint_vec_[joint_vec_.size() - 1]->joint_type =
            line.substr(name_index3 + 1, name_index4 - name_index3 - 1);
      }

      // Get the parent name
      index = line.find("<parent");
      if (index != -1 && is_joint == true) {
        int name_index1 = line.find("\"");
        int name_index2 = line.find("\"", name_index1 + 1);
        joint_vec_[joint_vec_.size() - 1]->parent_name =
            line.substr(name_index1 + 1, name_index2 - name_index1 - 1);
      }
      // Get the child name
      index = line.find("<child");
      if (index != -1 && is_joint == true) {
        int name_index1 = line.find("\"");
        int name_index2 = line.find("\"", name_index1 + 1);
        joint_vec_[joint_vec_.size() - 1]->child_name =
            line.substr(name_index1 + 1, name_index2 - name_index1 - 1);
      }

      index = line.find("/joint>");
      if (index != -1) {
        is_joint = false;
      }

      // Parse the origin for link-visual and joint
      index = line.find("<origin");
      if (index != -1) {
        int origin_index1 = line.find("\"");
        int origin_index2 = line.find("\"", origin_index1 + 1);
        std::string origin =
            line.substr(origin_index1 + 1, origin_index2 - origin_index1 - 1);

        std::string::size_type s1;
        std::string::size_type s2;
        double origin_x = std::stod(origin, &s1);
        double origin_y = std::stod(origin.substr(s1), &s2);
        double origin_z = std::stod((origin.substr(s1)).substr(s2));

        // Judge whether the origin is defined in the joint
        if (is_joint == true) {
          joint_vec_[joint_vec_.size() - 1]->origin[0] = origin_x;
          joint_vec_[joint_vec_.size() - 1]->origin[1] = origin_y;
          joint_vec_[joint_vec_.size() - 1]->origin[2] = origin_z;
        }
        // Judge whether the origin is defined in the link and in the visual
        // part
        else if (is_link == true && is_visual == true) {
          link_vec_[link_vec_.size() - 1]->origin[0] = origin_x;
          link_vec_[link_vec_.size() - 1]->origin[1] = origin_y;
          link_vec_[link_vec_.size() - 1]->origin[2] = origin_z;
        }
      }

      // Parse the rpy for link-visual and joint
      index = line.find("rpy");
      if(index != -1) {
        int rpy_index1 = line.find("\"", index + 1);
        int rpy_index2 = line.find("\"", rpy_index2 + 1);
        std::string rpy =
              line.substr(rpy_index1 + 1, rpy_index2 - rpy_index1 - 1);

          std::string::size_type s1;
          std::string::size_type s2;
          double rpy_x = std::stod(rpy, &s1);
          double rpy_y = std::stod(rpy.substr(s1), &s2);
          double rpy_z = std::stod((rpy.substr(s1)).substr(s2));

          // Judge whether the origin is defined in the joint
          if (is_joint == true) {
            joint_vec_[joint_vec_.size() - 1]->rpy[0] = rpy_x;
            joint_vec_[joint_vec_.size() - 1]->rpy[1] = rpy_y;
            joint_vec_[joint_vec_.size() - 1]->rpy[2] = rpy_z;
          }
          // Judge whether the origin is defined in the link and in the visual
          // part
          else if (is_link == true && is_visual == true) {
            link_vec_[link_vec_.size() - 1]->rpy[0] = rpy_x;
            link_vec_[link_vec_.size() - 1]->rpy[1] = rpy_y;
            link_vec_[link_vec_.size() - 1]->rpy[2] = rpy_z;
          }

      }

      // Parse the limit for joint
      index = line.find("<limit");
      if (index != -1 && is_joint == true) {
        int temp_index;
        temp_index = line.find("lower");
        int lower_index1 = line.find("\"", temp_index + 1);
        int lower_index2 = line.find("\"", lower_index1 + 1);
        std::string lower =
            line.substr(lower_index1 + 1, lower_index2 - lower_index1 - 1);

        temp_index = line.find("upper");
        int upper_index1 = line.find("\"", temp_index);
        int upper_index2 = line.find("\"", upper_index1 + 1);
        std::string upper =
            line.substr(upper_index1 + 1, upper_index2 - upper_index1 - 1);

        // joint_vec_[joint_vec_.size() - 1]->limit.has_limit = true;
        joint_vec_[joint_vec_.size() - 1]->limit[0] = std::stod(lower);
        joint_vec_[joint_vec_.size() - 1]->limit[1] = std::stod(upper);
      }

      // Parse the axis for joint
      index = line.find("<axis");
      if (index != -1 && is_joint == true) {
        int axis_index1 = line.find("\"");
        int axis_index2 = line.find("\"", axis_index1 + 1);
        std::string axis =
            line.substr(axis_index1 + 1, axis_index2 - axis_index1 - 1);

        std::string::size_type s1;
        std::string::size_type s2;
        double axis_x = std::stod(axis, &s1);
        double axis_y = std::stod(axis.substr(s1), &s2);
        double axis_z = std::stod((axis.substr(s1)).substr(s2));

        joint_vec_[joint_vec_.size() - 1]->axis[0] = axis_x;
        joint_vec_[joint_vec_.size() - 1]->axis[1] = axis_y;
        joint_vec_[joint_vec_.size() - 1]->axis[2] = axis_z;
      }
    }
  }

  // Construct the tree from link and joint
  // The root node should be "0:virtual"
  for (int i = 0; i <= joint_vec_.size() - 1; ++i) {
    std::string parent_name = joint_vec_[i]->parent_name;
    std::string children_name = joint_vec_[i]->child_name;
    Link* parent = name_link_map[parent_name];
    Link* children = name_link_map[children_name];

    // Add the articulation information
    children->joint_type = joint_vec_[i]->joint_type;
    children->joint_limit = joint_vec_[i]->limit;
    children->joint_axis = joint_vec_[i]->axis;
    // Convert the coordinate from the parent link frame to the children link
    // frame
    children->joint_origin = children->origin;
    children->joint_rpy = children->rpy;
    // children->joint_origin.x = children->origin.x;
    // children->joint_origin.y = children->origin.y;
    // children->joint_origin.z = children->origin.z;

    // Update the hierarchy tree
    children->parent_link = parent;
    parent->child_link.push_back(children);
    // std::cout <<parent_name << "; " << children_name << "\n";
  }

  fclose(stdin);

  // Get the root node of the URDF tree
  for (int i = 0; i <= link_vec_.size() - 1; ++i) {
    if (link_vec_[i]->parent_link == NULL) {
      root_ = link_vec_[i].get();
      break;
    }
  }

  if (root_ == NULL) {
    LOG(ERROR) << "Error format in URDF\n";
    return false;
  }

  return true;
}

}  // namespace assets
}  // namespace esp