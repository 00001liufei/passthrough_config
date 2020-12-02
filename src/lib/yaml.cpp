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

#include "yaml.hpp"

namespace YAML
{

	Node convert<passthrough_filter::passthroughConfig>::encode(const passthrough_filter::passthroughConfig &config)
	{
		Node node;
		node["rotation"] = Node(config.rotation);
		node["field"] = Node(config.field);
		return node;
	}
	bool convert<passthrough_filter::passthroughConfig>::decode(const Node &node, passthrough_filter::passthroughConfig &config)
	{
		config.rotation = node["rotation"].as<passthrough_filter::preRotation>();
		config.field = node["field"].as<std::vector<passthrough_filter::passthroughFieldConfig>>();
		return true;
	}

	Node convert<passthrough_filter::passthroughFieldConfig>::encode(const passthrough_filter::passthroughFieldConfig &config)
	{
		Node node;
		node["dim"] = Node(config.dim);
		node["min"] = Node(config.min);
		node["max"] = Node(config.max);
		return node;
	}
	bool convert<passthrough_filter::passthroughFieldConfig>::decode(const Node &node, passthrough_filter::passthroughFieldConfig &config)
	{
		config.dim = node["dim"].as<std::string>();
		config.min = node["min"].as<float>();
		config.max = node["max"].as<float>();
		return true;
	}

	Node convert<passthrough_filter::preRotation>::encode(const passthrough_filter::preRotation &config)
	{
		Node node;
		node["roll"] = Node(config.roll);
		node["pitch"] = Node(config.pitch);
		node["yaw"] = Node(config.yaw);
		return node;
	}
	bool convert<passthrough_filter::preRotation>::decode(const Node &node, passthrough_filter::preRotation &config)
	{
		config.roll = node["roll"].as<float>();
		config.pitch = node["pitch"].as<float>();
		config.yaw = node["yaw"].as<float>();
		return true;
	}



} // namespace YAML
