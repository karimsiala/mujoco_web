
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"

#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <emscripten/val.h>
#include <vector>

using namespace emscripten;

// main function
int main(int argc, char **argv) {
  std::printf("MuJoCo version: %d\n\n", 222);
  return 0;
}
