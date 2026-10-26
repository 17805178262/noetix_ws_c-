#ifndef NOETIX_ROBOT_FACTORY_HPP_
#define NOETIX_ROBOT_FACTORY_HPP_

#include <memory>

#include "noetix/config/robot_config.hpp"
#include "noetix/robot/robot_interface.hpp"

namespace noetix
{

/**
 * @brief Create the concrete RobotInterface from robot identity only.
 *
 * RobotFactory answers "which real robot is this?" and creates the matching
 * hardware implementation.
 */
std::unique_ptr<RobotInterface> CreateRobot(const RobotConfig& config);

} // namespace noetix

#endif // NOETIX_ROBOT_FACTORY_HPP_
