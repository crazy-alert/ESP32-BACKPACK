#include "../src/reset_button_policy.h"

using reset_button::Phase;
using reset_button::State;
using reset_button::step;

constexpr State boot_held = step(State{}, true, 0);
static_assert(boot_held.phase == Phase::WaitRelease, "held on boot must be ignored");
constexpr State idle = step(boot_held, false, 100);
static_assert(idle.phase == Phase::Idle, "release enables the button");
constexpr State pressed = step(idle, true, 200);
static_assert(step(pressed, false, 15199).phase == Phase::Idle, "short press must not reset");
static_assert(step(pressed, true, 15200).phase == Phase::Holding, "exactly 15s is not more than 15s");
constexpr State ready = step(pressed, true, 15201);
static_assert(ready.phase == Phase::Ready, "long press must arm reset");
static_assert(step(ready, true, 20000).phase == Phase::Ready, "do not reset while held");
static_assert(step(ready, false, 20001).phase == Phase::Fire, "release completes reset");
constexpr State near_wrap = step(idle, true, 0xfffffff0u);
static_assert(step(near_wrap, true, 14984u).phase == Phase::Holding, "millis wrap, 15s");
static_assert(step(near_wrap, true, 14985u).phase == Phase::Ready, "millis wrap, >15s");
