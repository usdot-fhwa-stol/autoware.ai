#pragma once

/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <iostream>
#include <vector>

namespace voxel_grid_filter
{

  /**
   * \brief Stuct containing the algorithm configuration values for <SUB><package_name>
   */
  struct Config
  { 
    std::string points_topic = "";
    bool _output_log = false;
    double measurement_range = 3.0;
  
    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "voxel_grid_filter::Config { " << std::endl
           << "points_topic: " << c.points_topic << std::endl
           << "output_log: " << c._output_log << std::endl
           << "measurement_range: " << c.measurement_range << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // voxel_grid_filter