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

#pragma once

#include <ros/ros.h>

#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <mujoco_ros/mujoco_env.hpp>

#include <array>
#include <string>
#include <vector>

namespace mujoco_ros::sensors {

/**
 * @brief Renders the reading of `force` sensors as a single arrow anchored at the sensor site.
 *
 * Unlike the built-in contact force visualization (viewer key 'F'), which draws one arrow per
 * contact point (e.g. four arrows for a box-shaped foot), this draws one resultant arrow per
 * sensor, which is much easier to read.
 */
class ForceVisualizerPlugin : public mujoco_ros::MujocoPlugin
{
public:
	ForceVisualizerPlugin()           = default;
	~ForceVisualizerPlugin() override = default;

	bool Load(const mjModel *model, mjData *data) override;

	void Reset() override;

	void RenderCallback(const mjModel *model, mjData *data, mjvScene *scene) override;

private:
	struct ForceArrow
	{
		int sensor_adr;
		int site_id;
		std::string sensor_name;
	};

	void applyVisualDefaults(const mjModel *model);
	bool configureFromParams();
	bool buildArrows(const mjModel *model);
	bool addArrowSource(const mjModel *model, int sensor_id);
	void appendArrow(const mjModel *model, mjData *data, mjvScene *scene, const ForceArrow &arrow) const;

	static bool readDoubleParam(const XmlRpc::XmlRpcValue &config, const std::string &name, double &value);
	static bool readBoolParam(const XmlRpc::XmlRpcValue &config, const std::string &name, bool &value);
	static std::vector<std::string> readStringArrayParam(const XmlRpc::XmlRpcValue &config, const std::string &name);
	static bool readRgbaParam(const XmlRpc::XmlRpcValue &config, const std::string &name, std::array<float, 4> &value);
	static double xmlRpcNumberToDouble(const XmlRpc::XmlRpcValue &value, double default_value);

	std::vector<ForceArrow> arrows_;
	std::vector<std::string> sensor_names_;

	// meters of arrow length per Newton
	double scale_ = 0.005;
	// arrow shaft radius in meters
	double width_ = 0.005;
	// forces below this magnitude (N) are not drawn
	double min_force_ = 1.0e-3;
	// clamp arrow length in meters, 0 disables clamping
	double max_length_ = 0.0;

	// MuJoCo's `force` sensor reports the force transmitted from the parent to the sensor's body.
	// For a foot sensor the ground reaction force is the negated value, hence the default.
	bool invert_ = true;

	std::array<float, 4> rgba_ = { 1.0f, 0.2f, 0.2f, 1.0f };
};

} // namespace mujoco_ros::sensors
