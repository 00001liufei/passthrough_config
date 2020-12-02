#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace passthrough_filter
{
    struct preRotation
    {
        float roll, pitch, yaw;
    };

    struct passthroughFieldConfig
    {
        std::string dim;
        float min, max;
    };

    struct passthroughConfig
    {
        preRotation rotation;
        std::vector<passthroughFieldConfig> field;
    };
} // namespace passthrough_filter