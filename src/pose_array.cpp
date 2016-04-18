/*
 * Copyright (c) 2016, Delft Robotics
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/validate_floats.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <QObject>

#include <boost/ptr_container/ptr_vector.hpp>


namespace dr {

namespace {

	bool validateFloats( const geometry_msgs::PoseArray& msg )
	{
		return rviz::validateFloats( msg.poses );
	}

	struct ShapeType {
		enum {
			Arrow,
			Axes,
		};
	};

	Ogre::Vector3 vectorRosToOgre(geometry_msgs::Point const & point) {
		return Ogre::Vector3(point.x, point.y, point.z);
	}

	Ogre::Quaternion quaternionRosToOgre(geometry_msgs::Quaternion const & quaternion) {
		return Ogre::Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
	}

	struct OgrePose {
		Ogre::Vector3 position;
		Ogre::Quaternion orientation;
	};

	template<typename T>
	std::string uniqueId(T * object) {
		unsigned int id = reinterpret_cast<std::size_t>(object);

		std::string result;
		result.reserve(16);
		while (id) {
			result += 'a' + (id % 26);
			id = id / 26;
		}
		return result;
	}
}

class PoseArrayDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PoseArray> {
Q_OBJECT
private:
	Ogre::SceneNode * arrow_node_;
	Ogre::SceneNode * axes_node_;

	std::vector<OgrePose> poses_;
	boost::ptr_vector<rviz::Arrow, boost::view_clone_allocator> arrows_;
	boost::ptr_vector<rviz::Axes,  boost::view_clone_allocator> axes_;
	bool arrows_dirty_;
	bool axes_dirty_;


	rviz::EnumProperty shape_property_;

	rviz::ColorProperty color_property_;
	rviz::FloatProperty alpha_property_;

	rviz::FloatProperty head_radius_property_;
	rviz::FloatProperty head_length_property_;
	rviz::FloatProperty shaft_radius_property_;
	rviz::FloatProperty shaft_length_property_;

	rviz::FloatProperty axes_length_property_;
	rviz::FloatProperty axes_radius_property_;

public:
	PoseArrayDisplay() :
		shape_property_("Shape", "Arrow", "Shape to display the pose as.", this, SLOT(updateShapeChoice())),
		color_property_("Color", QColor( 255, 25, 0 ), "Color to draw the arrows.", this, SLOT(updateColorAndAlpha())),
		alpha_property_("Alpha", 1, "Amount of transparency to apply to the arrow.", this, SLOT(updateColorAndAlpha())),
		head_radius_property_("Head Radius", 0.1, "Radius of the arrow's head, in meters.", this, SLOT(updateArrowGeometry())),
		head_length_property_("Head Length", 0.3, "Length of the arrow's head, in meters.", this, SLOT(updateArrowGeometry())),
		shaft_radius_property_("Shaft Radius", 0.05, "Radius of the arrow's shaft, in meters.", this, SLOT(updateArrowGeometry())),
		shaft_length_property_("Shaft Length", 1, "Length of the arrow's shaft, in meters.", this, SLOT(updateArrowGeometry())),
		axes_length_property_("Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxesGeometry())),
		axes_radius_property_("Axes Radius", 0.1, "Radius of each axis, in meters.", this, SLOT(updateAxesGeometry()))
	{
		//arrow_node_ = scene_node_->createChildSceneNode(); // "dr_pose_array_display_arrows_" + uniqueId(this));
		//axes_node_  = scene_node_->createChildSceneNode(); // "dr_pose_array_display_axes_" + uniqueId(this));

		shape_property_.addOption("Arrow", ShapeType::Arrow);
		shape_property_.addOption("Axes", ShapeType::Axes);
		alpha_property_.setMin(0);
		alpha_property_.setMax(1);

	}

	virtual ~PoseArrayDisplay() {}

protected:
	virtual void processMessage(const geometry_msgs::PoseArray::ConstPtr& msg) {
		if (!setTransform(msg->header)) return;

		poses_.resize(msg->poses.size());
		for (std::size_t i = 0; i < msg->poses.size(); ++i) {
			poses_[i].position    =     vectorRosToOgre(msg->poses[i].position);
			poses_[i].orientation = quaternionRosToOgre(msg->poses[i].orientation);
		}

		arrows_dirty_ = true;
		axes_dirty_   = true;
		updateObjects();
		context_->queueRender();
	}

	bool setTransform(std_msgs::Header const & header) {
		Ogre::Vector3 position;
		Ogre::Quaternion orientation;
		if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
			ROS_ERROR("Error transforming pose '%s' from frame '%s' to frame '%s'", qPrintable(getName()), header.frame_id.c_str(), qPrintable(fixed_frame_));
			return false;
		}
		scene_node_->setPosition(position);
		scene_node_->setOrientation(orientation);
		return true;
	}

	void updateObjects() {
		int shape = shape_property_.getOptionInt();

		if (shape == ShapeType::Arrow) {
			if (arrows_dirty_) updateArrows();
			arrows_dirty_ = false;
		} else if (shape == ShapeType::Axes) {
			if (axes_dirty_) updateAxes();
			axes_dirty_   = false;
		}
	}

	void updateArrows() {
		while (arrows_.size() < poses_.size()) arrows_.push_back(makeArrow());
		arrows_.resize(poses_.size(), NULL);

		Ogre::Quaternion adjust_orientation(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
		for (std::size_t i = 0; i < poses_.size(); ++i) {
			arrows_[i].setPosition(poses_[i].position);
			arrows_[i].setOrientation(poses_[i].orientation * adjust_orientation);
		}
	}

	void updateAxes() {
		while (axes_.size() < poses_.size()) axes_.push_back(makeAxes());
		axes_.resize(poses_.size(), NULL);
		for (std::size_t i = 0; i < poses_.size(); ++i) {
			axes_[i].setPosition(poses_[i].position);
			axes_[i].setOrientation(poses_[i].orientation);
		}
	}

	rviz::Arrow * makeArrow() {
		Ogre::ColourValue color = color_property_.getOgreColor();
		color.a                 = alpha_property_.getFloat();

		rviz::Arrow * arrow = new rviz::Arrow(
			scene_manager_,
			scene_node_,
			shaft_length_property_.getFloat(),
			shaft_radius_property_.getFloat(),
			head_length_property_.getFloat(),
			head_radius_property_.getFloat()
		);

		arrow->setColor(color);
		return arrow;
	}

	rviz::Axes * makeAxes() {
		return new rviz::Axes(
			scene_manager_,
			scene_node_,
			axes_length_property_.getFloat(),
			axes_radius_property_.getFloat()
		);
	}

private Q_SLOTS:
	void updateColorAndAlpha() {
		Ogre::ColourValue color = color_property_.getOgreColor();
		color.a                 = alpha_property_.getFloat();
		for (std::size_t i = 0; i < arrows_.size(); ++i) arrows_[i].setColor(color);
		context_->queueRender();
	}

	void updateShapeChoice() {
		int shape = shape_property_.getOptionInt();
		bool use_arrow = shape == ShapeType::Arrow;
		bool use_axes  = shape == ShapeType::Axes;

		color_property_.setHidden(!use_arrow);
		alpha_property_.setHidden(!use_arrow);
		shaft_length_property_.setHidden(!use_arrow);
		shaft_radius_property_.setHidden(!use_arrow);
		head_length_property_.setHidden(!use_arrow);
		head_radius_property_.setHidden(!use_arrow);

		axes_length_property_.setHidden(!use_axes);
		axes_radius_property_.setHidden(!use_axes);

		updateObjects();
		for (std::size_t i = 0; i < arrows_.size(); ++i) {
			arrows_[i].getSceneNode()->setVisible(use_arrow, true);
		}
		for (std::size_t i = 0; i < axes_.size(); ++i) {
			axes_[i].getSceneNode()->setVisible(use_axes, true);
		}
		context_->queueRender();
	}

	void updateArrowGeometry() {
		for (std::size_t i = 0; i < poses_.size(); ++i) {
			arrows_[i].set(
				shaft_length_property_.getFloat(),
				shaft_radius_property_.getFloat(),
				head_length_property_.getFloat(),
				head_radius_property_.getFloat()
			);
		}
		context_->queueRender();
	}

	void updateAxesGeometry() {
		for (std::size_t i = 0; i < poses_.size(); ++i) {
			axes_[i].set(
				axes_length_property_.getFloat(),
				axes_radius_property_.getFloat()
			);
		}
		context_->queueRender();
	}
};

}

#include "pose_array.moc"

PLUGINLIB_EXPORT_CLASS(dr::PoseArrayDisplay, rviz::Display);
