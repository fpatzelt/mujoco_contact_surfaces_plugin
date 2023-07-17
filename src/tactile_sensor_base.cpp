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

#include <mujoco_contact_surfaces_plugin/tactile_sensor_base.h>

namespace mujoco::plugin::contact_surfaces::sensors
{

	bool TactileSensorBase::load(const mjModel *m, mjData *d)
	{
		if (config_.IsMap() && config_["geomName"].IsDefined() && config_["updateRate"].IsDefined())
		{
			geomName = config_["geomName"].as<std::string>();

			int id = mj_name2id(m, mjOBJ_GEOM, geomName.c_str());
			if (id >= 0)
			{
				geomID = id;
				lastUpdate = d->time;
				updateRate = config_["updateRate"].as<double>();
				updatePeriod = 1.0 / updateRate;
				visualize = config_["visualize"].as<bool>();
				return true;
			}
		}
		return false;
	}

	void TactileSensorBase::update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions, int &sensor_adr)
	{
		double now = d->time;
		if (now < lastUpdate)
		{ // reset lastUpdate after jump back in time
			lastUpdate = d->time;
		}
		// if enough time has passed do a sensor update
		if (now - lastUpdate >= updatePeriod)
		{
			lastUpdate = now;
			n_vGeom = 0;
			int id = geomID;
			internal_update(m, d, geomCollisions, sensor_adr);
		}
	}

	void TactileSensorBase::renderCallback(const mjModel *model, mjData *data, mjvScene *scene)
	{
		if (visualize)
		{
			// ROS_WARN_STREAM_COND_NAMED(
			//     scene->maxgeom < n_vGeom, "mujoco_contact_surface_sensors",
			//     "Not all vgeoms could be visualized: n_vGeom = " << n_vGeom << " scene->maxgeom = " << scene->maxgeom);
			for (int i = 0; i < n_vGeom && scene->ngeom < scene->maxgeom; ++i)
			{
				scene->geoms[scene->ngeom++] = vGeoms[i];
			}
		}
	}

	void TactileSensorBase::reset() {}

	bool TactileSensorBase::initVGeom(int type, const mjtNum size[3], const mjtNum pos[3], const mjtNum mat[9],
									  const float rgba[4])
	{
		if (n_vGeom < mujoco::plugin::contact_surfaces::MAX_VGEOM)
		{
			mjvGeom *g = vGeoms + n_vGeom++;
			mjv_initGeom(g, type, size, pos, mat, rgba);
			return true;
		}
		return false;
	}
} // namespace mujoco::plugin::contact_surfaces::sensors