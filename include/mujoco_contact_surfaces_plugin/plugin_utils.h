/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
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
 *********************************************************************/

/* Authors: Florian Patzelt*/

#pragma once

// #include <ros/ros.h>
#include <mujoco_contact_surfaces_plugin/common_types.h>
#include <iostream>

// #include <pluginlib/class_loader.h>

namespace mujoco::plugin::contact_surfaces {
// using namespace MujocoSim;

class SurfacePlugin
{
public:
	virtual ~SurfacePlugin() = default;

	// Called directly after plugin creation
	void init(const YAML::Node &config)
	{
		config_ = config;
	};

	/**
	 * @brief Wrapper method that evaluates if loading the plugin is successful
	 *
	 * @param[in] m
	 * @param[out] d
	 * @return true if plugin could be loaded without errors.
	 * @return false if errors occurred during loading.
	 */
	bool safe_load(const mjModel * m, mjData * d)
	{
		loading_successful_ = load(m, d);
		if (!loading_successful_)
			// ROS_WARN_STREAM_NAMED("contact_surface_plugin",
			//                       "Plugin of type '"
			//                           << rosparam_config_["type"] << "' and full config '" << rosparam_config_
			//                           << "' failed to load. It will be ignored until the next load attempt.");
			std::cout << "contact_surface_plugin: " <<
			                      "Plugin of type '"
			                          << config_["type"].as<std::string>() << "' failed to load. It will be ignored until the next load attempt." << std::endl;
		return loading_successful_;
	}

	/**
	 * @brief Wrapper method that only calls reset if loading the plugin was successful.
	 */
	void safe_reset()
	{
		if (loading_successful_)
			reset();
	}

	/**
	 * @brief Override this callback to add custom behavior to handle geom collisions.
	 *
	 * @param[in] model pointer to mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] geomCollisions collisions between geoms.
	 */
	virtual void update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions, int &sensor_adr){};

	/**
	 * @brief Override this callback to add custom visualisations to the scene.
	 *
	 * @param[in] model pointer to mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] scene pointer to mjvScene.
	 */
	// virtual void renderCallback(mjModel * model, mjData * data, mjvScene *scene){};

	virtual int getSensorSize() = 0;

protected:
	/**
	 * @brief Called once the world is loaded.
	 *
	 * @param[in] m shared pointer to mujoco model.
	 * @param[in] d shared pointer to mujoco data.
	 * @return true on succesful load.
	 * @return false if load was not successful.
	 */
	virtual bool load(const mjModel * m, mjData * d) = 0;

	/**
	 * @brief Called on reset.
	 */
	virtual void reset() = 0;



private:
	bool loading_successful_ = false;

protected:
	SurfacePlugin() {};
	YAML::Node config_;
};

} // namespace mujoco::plugin::contact_surfaces
