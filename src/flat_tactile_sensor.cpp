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

#include <mujoco_contact_surfaces_plugin/flat_tactile_sensor.h>

namespace mujoco::plugin::contact_surfaces::sensors
{
	using namespace drake;
	using namespace drake::geometry;

	bool FlatTactileSensor::load(const mjModel *m, mjData *d)
	{
		if (TactileSensorBase::load(m, d) && config_["resolution"].IsDefined()) {
		 	resolution = config_["resolution"].as<double>();

			double xs = m->geom_size[3 * geomID];
			double ys = m->geom_size[3 * geomID + 1];
			cx        = (int)(2 * xs / resolution);
			cy        = (int)(2 * ys / resolution);
			vGeoms    = new mjvGeom[cx * cy];
			std::cout << "mujoco_contact_surface_sensors: " << 
			                      "Found tactile sensor array of size " << cx << "x" << cy << std::endl;		
			return true;
		}
		return false;
	}

	void FlatTactileSensor::internal_update(const mjModel *m, mjData *d,
											const std::vector<GeomCollisionPtr> &geomCollisions, int &sensor_adr)
	{
		if (visualize)
		{
			// reset the visualized geoms
			tactile_running_scale = 0.9 * tactile_running_scale + 0.1 * tactile_current_scale;
			tactile_current_scale = 0.;
		}
		mjtNum *sensordata = d->sensordata + sensor_adr;
		int id = geomID;
		double xs = m->geom_size[3 * id];
		double ys = m->geom_size[3 * id + 1];
		double zs = m->geom_size[3 * id + 2];
		double res = resolution;
		Eigen::Matrix4d M, Minv, Mback;
		M << d->geom_xmat[9 * id + 0], d->geom_xmat[9 * id + 1], d->geom_xmat[9 * id + 2], d->geom_xpos[3 * id],
			d->geom_xmat[9 * id + 3], d->geom_xmat[9 * id + 4], d->geom_xmat[9 * id + 5], d->geom_xpos[3 * id + 1],
			d->geom_xmat[9 * id + 6], d->geom_xmat[9 * id + 7], d->geom_xmat[9 * id + 8], d->geom_xpos[3 * id + 2], 0, 0, 0,
			1;

		Minv = M.inverse();
		Minv.col(3) << Minv.col(3) + Eigen::Vector4d(xs, ys, -zs, 0);
		if (visualize)
		{
			Mback = Minv.inverse();
		}
		std::vector<double> pressure[cx][cy];

		mjtNum size[3] = {res / 2, res / 2, 0.001};
		mjtNum rot[9];
		for (int i = 0; i < 9; ++i)
		{
			rot[i] = d->geom_xmat[9 * id + i];
		}

		for (GeomCollisionPtr gc : geomCollisions)
		{
			if (gc->g1 == id or gc->g2 == id)
			{
				std::shared_ptr<ContactSurface<double>> s = gc->s;
				auto mesh = s->tri_mesh_W();

				// prepare caches
				const int n = mesh.num_elements();

				// get geom transformation
				for (int t = 0; t < n; ++t)
				{
					// project points onto 2d sensor plane
					std::vector<Eigen::Vector2d> tpoints = {};
					auto element = mesh.element(t);
					for (int i = 0; i < element.num_vertices(); ++i)
					{
						int v = element.vertex(i);
						const Vector3<double> &vp = mesh.vertex(v);
						const Eigen::Vector4d vpe(vp[0], vp[1], vp[2], 1);
						Eigen::Vector4d vpp = Minv * vpe;

						tpoints.push_back(Eigen::Vector2d(vpp[0], vpp[1]));
					}
					int st0 = (int)((tpoints[1] - tpoints[0]).norm() / res * 2.) + 1;
					int st1 = (int)((tpoints[2] - tpoints[0]).norm() / res * 2.) + 1;
					int st2 = (int)((tpoints[2] - tpoints[0]).norm() / res * 2.) + 1;
					int st = std::max(st0, st1);
					for (double a = 0; a <= 1; a += 1. / st)
					{
						for (double b = 0; b <= 1; b += 1. / st2)
						{
							const Vector3<double> bary(a, (1 - a) * (1 - b), (1 - a) * b);
							Eigen::Vector2d p = bary[0] * tpoints[0] + bary[1] * tpoints[1] + bary[2] * tpoints[2];
							if (p[0] > 0 && p[0] < 2 * xs && p[1] > 0 && p[1] < 2 * ys)
							{
								int x = (int)std::floor(p[0] / res);
								int y = (int)std::floor(p[1] / res);
								pressure[x][y].push_back(s->tri_e_MN().Evaluate(t, bary) * s->area(t));
							}
						}
					}
				}
			}
		}

		for (int x = 0; x < cx; ++x)
		{
			for (int y = 0; y < cy; ++y)
			{
				int nt = pressure[x][y].size();
				if (nt > 0)
				{
					double mp = 0;

					for (int i = 0; i < nt; ++i)
					{
						mp += pressure[x][y][i];
					}

					double p0 = mp / nt;
					sensordata[x * cx + y] = p0;
					if (visualize) {
						tactile_current_scale = std::max(std::abs(p0), tactile_current_scale);
						float ps              = std::min(std::abs(p0), tactile_running_scale) / tactile_running_scale;

						const float rgba[4] = { ps, 0, (1.f - ps), 0.8 };
						Eigen::Vector4d dp  = Mback * Eigen::Vector4d(x * res + res / 2, y * res + res / 2, 0, 1);
						mjtNum pos[3]       = { dp[0], dp[1], dp[2] };

						mjvGeom *g = vGeoms + n_vGeom++;
						mjv_initGeom(g, mjGEOM_BOX, size, pos, rot, rgba);
					}
				}
				else
				{
					sensordata[x * cx + y] = 0;
				}
			}
		}
	}

	int FlatTactileSensor::getSensorSize()
	{
		return cx * cy;
	}

} // namespace mujoco::plugin::contact_surfaces::sensors
