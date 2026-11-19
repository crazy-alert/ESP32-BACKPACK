#pragma once
#include <stdint.h>
namespace reset_button {
enum class Phase { WaitRelease, Idle, Holding, Ready, Fire };
struct State {
  Phase phase;
  uint32_t started;
  constexpr State(Phase p = Phase::WaitRelease, uint32_t t = 0) : phase(p), started(t) {}
};
constexpr State step(State state, bool pressed, uint32_t now) {
  return state.phase == Phase::WaitRelease ? (pressed ? state : State(Phase::Idle)) :
         state.phase == Phase::Idle ? (pressed ? State(Phase::Holding, now) : state) :
         state.phase == Phase::Holding ? (!pressed ? State(Phase::Idle) :
             static_cast<uint32_t>(now - state.started) > 15000u ? State(Phase::Ready, state.started) : state) :
         state.phase == Phase::Ready ? (pressed ? state : State(Phase::Fire, state.started)) : state;
}
}
