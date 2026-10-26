/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/console.h>

#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/properties/vector_property.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

#include <OgreMaterial.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <cmath> // for atan2 and constants

#include "plant_flag_tool.h"

namespace rviz_plugin_tutorials {


void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
  // Get the number of poses in the path
  int num_poses = msg->poses.size();
  ROS_INFO("Received path with %d poses harold riascos", num_poses);

  // Print the first pose (if available)
  if (num_poses > 0) {
    geometry_msgs::PoseStamped first_pose = msg->poses[3];
    ROS_INFO("First pose position: x=%.2f, y=%.2f, z=%.2f",
             first_pose.pose.position.x, first_pose.pose.position.y,
             first_pose.pose.position.z);
  }
}


double calculateAngle(double x, double y) {
    // atan2 returns angle in radians between [-π, π]
    double angle_rad = std::atan2(y, x);
    double RAD_TO_DEG = 180.0 / M_PI;
    
    // Convert to degrees [0, 360)
    double angle_deg = angle_rad * RAD_TO_DEG;
    if (angle_deg < 0) {
        angle_deg += 360;
    }
    
    return angle_deg;
}
// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
PlantFlagTool::PlantFlagTool()
    : moving_flag_node_(NULL), current_flag_property_(NULL) {
  shortcut_key_ = 'l';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
PlantFlagTool::~PlantFlagTool() {
  for (unsigned i = 0; i < flag_nodes_.size(); i++) {
    scene_manager_->destroySceneNode(flag_nodes_[i]);
  }
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void PlantFlagTool::onInitialize() {
  // flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";
  flag_resource_ = "package://inspection_control_software/media/arrow.dae";

  if (rviz::loadMeshFromResource(flag_resource_).isNull()) {
    ROS_ERROR("PlantFlagTool: failed to load model resource '%s'.",
              flag_resource_.c_str());
    return;
  }

  moving_flag_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
  // Ogre::Entity *entity = scene_manager_->createEntity("sphere",
  // Ogre::SceneManager::PT_SPHERE);
  moving_flag_node_->attachObject(entity);
  moving_flag_node_->setVisible(false);
  moving_flag_node_->setScale(Ogre::Vector3(0.01, 0.01, 0.01));

  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  pose_pub_ = nh.advertise<nav_msgs::Path>("/target_pose", 3, true);
  // sub_ = nh2.subscribe<nav_msgs::Path>("/robot_path", 2, &pathCallback);
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void PlantFlagTool::activate() {
  if (moving_flag_node_) {
    moving_flag_node_->setVisible(true);

    // current_flag_property_ =
        // new rviz::VectorProperty("Flag " + QString::number(flag_nodes_.size()));
    // current_flag_property_->setReadOnly(true);
    // getPropertyContainer()->addChild(current_flag_property_);
  }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.
void PlantFlagTool::deactivate() {
  if (moving_flag_node_) {
    moving_flag_node_->setVisible(false);
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int PlantFlagTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (!moving_flag_node_) {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Vector3 intersection_saved(0, 0, 0);

  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                        event.y, intersection)) {
    // moving_flag_node_->setVisible(true);
    // moving_flag_node_->setPosition(intersection);
    // current_flag_property_->setVector(intersection);
    if (state_ == 1) {
      // moving_flag_node_->roll(Ogre::Degree(60), Ogre::Node::TS_WORLD);
      float dx = intersection.x - intersection_saved.x;
      float dy = intersection.y - intersection_saved.y;
      float angle = calculateAngle(dx, dy);
      moving_flag_node_->setVisible(true);
      // moving_flag_node_->setOrientation(Ogre::Quaternion(Ogre::Degree(angle), Ogre::Vector3(0,1,1)));
      // moving_flag_node_->lookAt(Ogre::Vector3(dx, dy, 0), Ogre::Node::TS_WORLD);
      moving_flag_node_->setDirection(dx, dy, 0);
    }

    if (event.left()) {
      intersection_saved = intersection;
      state_ = 1;
      moving_flag_node_->setPosition(intersection);
    }

    if (event.left()) {
      makeFlag(intersection_saved);
      current_flag_property_ = NULL;
      // Drop the reference so that deactivate() won't remove it.
      return Render | Finished;
    }
  } else {
    moving_flag_node_->setVisible(false); // If the mouse is not pointing at the
                                          // ground plane, don't show the flag.
  }
  return Render;
}

// This is a helper function to create a new flag in the Ogre scene and save its
// scene node in a list.
// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void PlantFlagTool::save(rviz::Config config) const {
  config.mapSetValue("Class", getClassId());

  // The top level of this tool's Config is a map, but our flags
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``flags_config``) to store
  // the list.
  rviz::Config flags_config = config.mapMakeChild("Flags");

  // To read the positions and names of the flags, we loop over the
  // the children of our Property container:
  rviz::Property *container = getPropertyContainer();
  int num_children = container->numChildren();
  for (int i = 0; i < num_children; i++) {
    rviz::Property *position_prop = container->childAt(i);
    // For each Property, we create a new Config object representing a
    // single flag and append it to the Config list.
    rviz::Config flag_config = flags_config.listAppendNew();
    // Into the flag's config we store its name:
    flag_config.mapSetValue("Name", position_prop->getName());
    // ... and its position.
    position_prop->save(flag_config);
  }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void PlantFlagTool::load(const rviz::Config &config) {
  // Here we get the "Flags" sub-config from the tool config and loop over its
  // entries:
  rviz::Config flags_config = config.mapGetChild("Flags");
  int num_flags = flags_config.listLength();
  for (int i = 0; i < num_flags; i++) {
    rviz::Config flag_config = flags_config.listChildAt(i);
    // At this point each ``flag_config`` represents a single flag.
    //
    // Here we provide a default name in case the name is not in the config file
    // for some reason:
    QString name = "Flag " + QString::number(i + 1);
    // Then we use the convenience function mapGetString() to read the
    // name from ``flag_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    flag_config.mapGetString("Name", &name);
    // Given the name we can create an rviz::VectorProperty to display the
    // position:
    rviz::VectorProperty *prop = new rviz::VectorProperty(name);
    // Then we just tell the property to read its contents from the config, and
    // we've read all the data.
    prop->load(flag_config);
    // We finish each flag by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible flag object in the 3D scene at the correct
    // position.
    prop->setReadOnly(true);
    getPropertyContainer()->addChild(prop);
    makeFlag(prop->getVector());
  }
}

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::PlantFlagTool, rviz::Tool)
// END_TUTORIAL
