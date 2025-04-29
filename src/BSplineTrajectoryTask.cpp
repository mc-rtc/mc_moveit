#include <mc_moveit/BSplineTrajectoryTask.h>

#include <mc_rtc/gui/Trajectory.h>

namespace mc_moveit
{

void BSplineTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Checkbox("Paused", [this]() { return paused_; }, [this]() { paused_ = !paused_; }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("Surface pose", [this]() { return frame_->position(); }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("Target", [this]() { return target(); }));
  auto samples = spline().sampleTrajectory();
  mc_rtc::gui::LineConfig lconfig;
  lconfig.color = mc_rtc::gui::Color::Green;
  lconfig.width = 0.05;
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Trajectory("Trajectory", lconfig,
                                         [samples]() -> const std::vector<Eigen::Vector3d> & { return samples; }));
}

} // namespace mc_moveit
