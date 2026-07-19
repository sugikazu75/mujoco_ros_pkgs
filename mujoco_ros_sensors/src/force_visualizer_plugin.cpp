/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2025, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <mujoco_ros_sensors/force_visualizer_plugin.h>

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>

namespace mujoco_ros::sensors {

bool ForceVisualizerPlugin::Load(const mjModel *model, mjData * /*data*/)
{
	ROS_INFO_NAMED("force_visualizer", "Loading force visualizer plugin ...");

	applyVisualDefaults(model);
	if (!configureFromParams()) {
		return false;
	}

	arrows_.clear();
	if (!buildArrows(model) || arrows_.empty()) {
		ROS_ERROR_NAMED("force_visualizer", "No force sensors were found for visualization.");
		return false;
	}

	ROS_INFO_STREAM_NAMED("force_visualizer", "Loaded force visualizer with " << arrows_.size() << " arrow(s), scale="
	                                                                          << scale_ << ", width=" << width_ << ".");
	return true;
}

void ForceVisualizerPlugin::Reset() {}

void ForceVisualizerPlugin::RenderCallback(const mjModel *model, mjData *data, mjvScene *scene)
{
	for (const auto &arrow : arrows_) {
		if (scene->ngeom >= scene->maxgeom) {
			mj_warning(data, mjWARN_VGEOMFULL, scene->maxgeom);
			return;
		}

		appendArrow(model, data, scene, arrow);
	}
}

void ForceVisualizerPlugin::applyVisualDefaults(const mjModel *model)
{
	// same derivation as MuJoCo's built-in contact force arrows
	scale_ = model->vis.map.force / std::max(static_cast<mjtNum>(mjMINVAL), model->stat.meanmass);
	width_ = model->stat.meansize * model->vis.scale.forcewidth;
	if (width_ <= 0.0) {
		width_ = model->vis.scale.forcewidth;
	}

	for (int i = 0; i < 4; ++i) {
		rgba_[i] = model->vis.rgba.contactforce[i];
	}
}

bool ForceVisualizerPlugin::configureFromParams()
{
	readDoubleParam(rosparam_config_, "scale", scale_);
	readDoubleParam(rosparam_config_, "width", width_);
	readDoubleParam(rosparam_config_, "min_force", min_force_);
	readDoubleParam(rosparam_config_, "max_length", max_length_);
	readBoolParam(rosparam_config_, "invert", invert_);
	readRgbaParam(rosparam_config_, "rgba", rgba_);

	sensor_names_ = readStringArrayParam(rosparam_config_, "sensor_names");

	if (scale_ <= 0.0) {
		ROS_ERROR_NAMED("force_visualizer", "`scale` must be positive.");
		return false;
	}

	if (width_ <= 0.0) {
		ROS_ERROR_NAMED("force_visualizer", "`width` must be positive.");
		return false;
	}

	min_force_  = std::max(0.0, min_force_);
	max_length_ = std::max(0.0, max_length_);

	return true;
}

bool ForceVisualizerPlugin::buildArrows(const mjModel *model)
{
	if (sensor_names_.empty()) {
		// no explicit selection: visualize every force sensor in the model
		for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
			if (model->sensor_type[sensor_id] == mjSENS_FORCE) {
				addArrowSource(model, sensor_id);
			}
		}
		return !arrows_.empty();
	}

	bool success = true;
	for (const auto &sensor_name : sensor_names_) {
		const int sensor_id = mj_name2id(const_cast<mjModel *>(model), mjOBJ_SENSOR, sensor_name.c_str());
		if (sensor_id < 0) {
			ROS_ERROR_STREAM_NAMED("force_visualizer", "Sensor `" << sensor_name << "` was not found.");
			success = false;
			continue;
		}

		if (model->sensor_type[sensor_id] != mjSENS_FORCE) {
			ROS_ERROR_STREAM_NAMED("force_visualizer", "Sensor `" << sensor_name << "` is not a force sensor.");
			success = false;
			continue;
		}

		success &= addArrowSource(model, sensor_id);
	}

	return success;
}

bool ForceVisualizerPlugin::addArrowSource(const mjModel *model, int sensor_id)
{
	const char *sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, sensor_id);

	if (model->sensor_objtype[sensor_id] != mjOBJ_SITE) {
		ROS_ERROR_STREAM_NAMED("force_visualizer",
		                       "Force sensor `" << (sensor_name ? sensor_name : "") << "` is not attached to a site.");
		return false;
	}

	arrows_.push_back({ model->sensor_adr[sensor_id], model->sensor_objid[sensor_id], sensor_name ? sensor_name : "" });
	return true;
}

void ForceVisualizerPlugin::appendArrow(const mjModel *model, mjData *data, mjvScene *scene,
                                        const ForceArrow &arrow) const
{
	// force sensor values are expressed in the site frame
	mjtNum force_local[3];
	mju_copy3(force_local, data->sensordata + arrow.sensor_adr);
	if (invert_) {
		mju_scl3(force_local, force_local, -1.);
	}

	const mjtNum magnitude = mju_norm3(force_local);
	if (magnitude < min_force_) {
		return;
	}

	mjtNum length = magnitude * scale_;
	if (max_length_ > 0.0) {
		length = std::min(length, static_cast<mjtNum>(max_length_));
	}

	const mjtNum *site_pos = data->site_xpos + 3 * arrow.site_id;
	const mjtNum *site_mat = data->site_xmat + 9 * arrow.site_id;

	mjtNum dir_local[3];
	mju_scl3(dir_local, force_local, 1. / magnitude);

	mjtNum dir_world[3];
	mju_mulMatVec3(dir_world, site_mat, dir_local);

	mjtNum tip[3];
	mju_addScl3(tip, site_pos, dir_world, length);

	mjvGeom *geom = scene->geoms + scene->ngeom++;
	mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba_.data());
	mjv_connector(geom, mjGEOM_ARROW, static_cast<mjtNum>(width_), site_pos, tip);
}

bool ForceVisualizerPlugin::readDoubleParam(const XmlRpc::XmlRpcValue &config, const std::string &name, double &value)
{
	if (!config.hasMember(name)) {
		return false;
	}

	value = xmlRpcNumberToDouble(config[name], value);
	return true;
}

bool ForceVisualizerPlugin::readBoolParam(const XmlRpc::XmlRpcValue &config, const std::string &name, bool &value)
{
	if (!config.hasMember(name) || config[name].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
		return false;
	}

	value = static_cast<bool>(config[name]);
	return true;
}

std::vector<std::string> ForceVisualizerPlugin::readStringArrayParam(const XmlRpc::XmlRpcValue &config,
                                                                     const std::string &name)
{
	std::vector<std::string> values;
	if (!config.hasMember(name)) {
		return values;
	}

	const XmlRpc::XmlRpcValue &xml_values = config[name];
	if (xml_values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_WARN_STREAM_NAMED("force_visualizer", "`" << name << "` must be a string array. Ignoring it.");
		return values;
	}

	for (int i = 0; i < xml_values.size(); ++i) {
		if (xml_values[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
			ROS_WARN_STREAM_NAMED("force_visualizer", "`" << name << "` contains a non-string value. Ignoring it.");
			values.clear();
			return values;
		}
		values.push_back(static_cast<std::string>(xml_values[i]));
	}

	return values;
}

bool ForceVisualizerPlugin::readRgbaParam(const XmlRpc::XmlRpcValue &config, const std::string &name,
                                          std::array<float, 4> &value)
{
	if (!config.hasMember(name)) {
		return false;
	}

	const XmlRpc::XmlRpcValue &rgba = config[name];
	if (rgba.getType() != XmlRpc::XmlRpcValue::TypeArray || rgba.size() != 4) {
		ROS_WARN_STREAM_NAMED("force_visualizer", "`" << name << "` must be an array with 4 numbers. Ignoring it.");
		return false;
	}

	for (int i = 0; i < 4; ++i) {
		value[i] = static_cast<float>(xmlRpcNumberToDouble(rgba[i], value[i]));
	}

	return true;
}

double ForceVisualizerPlugin::xmlRpcNumberToDouble(const XmlRpc::XmlRpcValue &value, double default_value)
{
	if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
		return static_cast<double>(value);
	}
	if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
		return static_cast<int>(value);
	}
	return default_value;
}

} // namespace mujoco_ros::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::ForceVisualizerPlugin, mujoco_ros::MujocoPlugin)
