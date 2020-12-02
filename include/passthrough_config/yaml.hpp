/*
  multi_sensor_calibration
  Copyright (C) 2019  Intelligent Vehicles Delft University of Technology

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#pragma once
#include <yaml-cpp/yaml.h>
#include "config.hpp"

namespace YAML
{

	template <>
	struct convert<passthrough_filter::passthroughConfig>
	{
		static Node encode(const passthrough_filter::passthroughConfig &config);
		static bool decode(const Node &node, passthrough_filter::passthroughConfig &config);
	};

	template <>
	struct convert<passthrough_filter::preRotation>
	{
		static Node encode(const passthrough_filter::preRotation &config);
		static bool decode(const Node &node, passthrough_filter::preRotation &config);
	};

	template <>
	struct convert<passthrough_filter::passthroughFieldConfig>
	{
		static Node encode(const passthrough_filter::passthroughFieldConfig &config);
		static bool decode(const Node &node, passthrough_filter::passthroughFieldConfig &config);
	};

	

} // namespace YAML
