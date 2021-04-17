/*!
 *      @file  parameters.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#pragma once

#include <vector>
#include <string>

namespace mavsdk_ros {
class Parameters {
public:
    Parameters();

    int local_system_id;
    int local_component_id;

    std::vector<int> sensor_components_ids;

    std::string local_ip;
    std::string target_ip;

    int local_port;
    int target_port;

    int target_system_id;

private:
    void exitWithParameterError(const char* parameterStr);
};
} // namespace mavsdk_ros