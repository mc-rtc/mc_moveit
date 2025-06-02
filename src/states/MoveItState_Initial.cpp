#include "MoveItState_Initial.h"

#include "../MoveItFSM.h"

void MoveItState_Initial::configure(const mc_rtc::Configuration & config)
{
}

void MoveItState_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MoveItFSM &>(ctl_);
}

bool MoveItState_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MoveItFSM &>(ctl_);
  output("OK");
  return true;
}

void MoveItState_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MoveItFSM &>(ctl_);
}

EXPORT_SINGLE_STATE("MoveItState_Initial", MoveItState_Initial)
