#pragma once

#include "planning_service/service/utils/resource_registry.h"

namespace service {
namespace utils {
namespace web {

/** @brief Generate HTML for the Meshcat ports table from the Draco map.
 *
 * This function generates an HTML page that lists all the Meshcat ports
 * associated with the Draco planners in the provided map.
 *
 * @param draco_map Map of Draco planners keyed by their unique IDs.
 */
void MakeMeshcatPortsHtml(
    const std::map<uint64_t, std::unique_ptr<draco::planner::DracoPlanner>>&
        draco_map,
    std::atomic<bool>& stop_requested, std::mutex& html_mutex,
    std::condition_variable& html_cv, int port_number = 7777);

}  // namespace web
}  // namespace utils
}  // namespace service
