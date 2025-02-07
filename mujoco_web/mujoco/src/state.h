#pragma once

#include <cstdio>
#include <cstring>

#include "mujoco/mujoco.h"

#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <emscripten/val.h>

#include <model.h>

using namespace emscripten;

/**
 * @brief Represents the simulation state for a MuJoCo model.
 *
 * The `State` class encapsulates the dynamic simulation data (`mjData`)
 * associated with a given MuJoCo model (`mjModel`). It provides methods
 * to access and manage the simulation state, enabling interaction with
 * Emscripten and JavaScript through bindings.
 * @param m The MuJoCo model associated with this state.
 *
 * ## Usage Example
 *
 * ```cpp
 * #include "State.h"
 *
 * int main() {
 * // Load a model
 * Model model = Model::load_from_xml("path/to/model.xml");
 *
 * // Initialize the simulation state
 * State state(model);
 *
 * // Access simulation data
 * mjData* dataPtr = state.ptr();
 *
 * // Perform simulation steps...
 *
 * // Clean up
 * state.free();
 *
 * return 0;
 * }
 */
class State {
public:
  State(Model m) { d = mj_makeData(m.ptr()); }
  mjData *ptr() { return d; }
  mjData getVal() { return *d; }
  void free() { return mju_free(d); }

private:
  mjData *d;
};