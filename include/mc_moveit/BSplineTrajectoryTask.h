#pragma once

#include <mc_moveit/api.h>

#include <mc_tasks/BSplineTrajectoryTask.h>

namespace mc_moveit
{

/** This task derives from mc_tasks::BSplineTrajectoryTask
 *
 * The difference lies in the GUI, instead of displaying editable waypoints we display the trajectory
 */
struct MC_MOVEIT_DLLAPI BSplineTrajectoryTask : public mc_tasks::BSplineTrajectoryTask
{
  using mc_tasks::BSplineTrajectoryTask::BSplineTrajectoryTask;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
};

} // namespace mc_moveit
