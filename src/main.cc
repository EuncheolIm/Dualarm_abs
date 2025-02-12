// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

// #include "controller.h"
#include "controller_abs.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

using Seconds = std::chrono::duration<double>;
CController TaskController(15);

void visualize_state_planner(mj::Simulate& sim)
{
  // // std::cout << FUNC_TAG << std::endl;
  // if(count == 1)
  // {
  //   for(int i = 0; i < 3; ++i)
  //   {
  //     sim.ee_pos[i] = TaskControl._x_hand(i);
  //     sim.ee_pos_d[i] = TaskControl._x_hand(i);
  //   }
  // }
  // else {
  //   for(int i = 0; i < 3; ++i)
  //   {
  //     sim.ee_pos[i] = TaskControl._x_hand(i);
  //     sim.ee_pos_d[i] = TaskControl._x_des_hand(i);
  //   }
  // }
  // count ++;

  for(int i = 0; i < 3; ++i)
  {
    sim.ee_pos_left[i] = TaskController.Model._x_left_hand(i);
    // sim.ee_pos_d_left[i] = TaskController._x_des_left_hand(i);

    sim.ee_pos_right[i] = TaskController.Model._x_right_hand(i);
    // sim.ee_pos_d_right[i] = TaskController._x_des_right_hand(i);
  }

  // 먼저 메모리를 할당하지 않고 배열 복사
  mjtNum newarray_ee_left[3];
  mjtNum newarray_ee_d_left[3];
  mjtNum newarray_ee_right[3];
  mjtNum newarray_ee_d_right[3];

  std::copy(std::begin(sim.ee_pos_left), std::end(sim.ee_pos_left), newarray_ee_left);
  // std::copy(std::begin(sim.ee_pos_d_left), std::end(sim.ee_pos_d_left), newarray_ee_d_left);
  std::copy(std::begin(sim.ee_pos_right), std::end(sim.ee_pos_right), newarray_ee_right);
  // std::copy(std::begin(sim.ee_pos_d_right), std::end(sim.ee_pos_d_right), newarray_ee_d_right);

  if(sim.ee_traj_left.size() < sim.history) {
      sim.ee_traj_left.push_back(new mjtNum[3]{newarray_ee_left[0], newarray_ee_left[1], newarray_ee_left[2]});
      sim.ee_traj_d_left.push_back(new mjtNum[3]{newarray_ee_d_left[0], newarray_ee_d_left[1], newarray_ee_d_left[2]});
      sim.ee_traj_right.push_back(new mjtNum[3]{newarray_ee_right[0], newarray_ee_right[1], newarray_ee_right[2]});
      sim.ee_traj_d_right.push_back(new mjtNum[3]{newarray_ee_d_right[0], newarray_ee_d_right[1], newarray_ee_d_right[2]});
  }
  else if(sim.ee_traj_left.size() == sim.history) {
      delete[] sim.ee_traj_left.front();
      delete[] sim.ee_traj_d_left.front();
      delete[] sim.ee_traj_right.front();
      delete[] sim.ee_traj_d_right.front();

      sim.ee_traj_left.pop_front();
      sim.ee_traj_d_left.pop_front();
      sim.ee_traj_right.pop_front();
      sim.ee_traj_d_right.pop_front();

      sim.ee_traj_left.push_back(new mjtNum[3]{newarray_ee_left[0], newarray_ee_left[1], newarray_ee_left[2]});
      sim.ee_traj_d_left.push_back(new mjtNum[3]{newarray_ee_d_left[0], newarray_ee_d_left[1], newarray_ee_d_left[2]});
      sim.ee_traj_right.push_back(new mjtNum[3]{newarray_ee_right[0], newarray_ee_right[1], newarray_ee_right[2]});
      sim.ee_traj_d_right.push_back(new mjtNum[3]{newarray_ee_d_right[0], newarray_ee_d_right[1], newarray_ee_d_right[2]});
  }

  // mjtNum* newarray_ee_left = new mjtNum[3];
  // mjtNum* newarray_ee_d_left = new mjtNum[3];

  // mjtNum* newarray_ee_right = new mjtNum[3];
  // mjtNum* newarray_ee_d_right = new mjtNum[3];

  // std::copy(std::begin(sim.ee_pos_left), std::end(sim.ee_pos_left), newarray_ee_left);
  // std::copy(std::begin(sim.ee_pos_d_left), std::end(sim.ee_pos_d_left), newarray_ee_d_left);
  // std::copy(std::begin(sim.ee_pos_right), std::end(sim.ee_pos_right), newarray_ee_right);
  // std::copy(std::begin(sim.ee_pos_d_right), std::end(sim.ee_pos_d_right), newarray_ee_d_right);

  // if(sim.ee_traj_left.size() < sim.history) {
  //   sim.ee_traj_left.push_back(newarray_ee_left);
  //   sim.ee_traj_d_left.push_back(newarray_ee_d_left);
  //   sim.ee_traj_right.push_back(newarray_ee_right);
  //   sim.ee_traj_d_right.push_back(newarray_ee_d_right);

  // }
  // else if(sim.ee_traj_left.size() == sim.history) {
  //   sim.ee_traj_left.push_back(newarray_ee_left);
  //   sim.ee_traj_d_left.push_back(newarray_ee_d_left);
  //   sim.ee_traj_right.push_back(newarray_ee_right);
  //   sim.ee_traj_d_right.push_back(newarray_ee_d_right);

  //   delete[] sim.ee_traj_left.front();
  //   delete[] sim.ee_traj_d_left.front();
  //   delete[] sim.ee_traj_right.front();
  //   delete[] sim.ee_traj_d_right.front();

  //   sim.ee_traj_left.pop_front();
  //   sim.ee_traj_d_left.pop_front();
  //   sim.ee_traj_right.pop_front();
  //   sim.ee_traj_d_right.pop_front();

  // }
}


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  // if no error and load took more than 1/4 seconds, report load time
  if (!loadError[0] && load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  int body_id = mj_name2id(m, mjOBJ_BODY, "object_box1");
    if (body_id == -1) {
        printf("Body not found.\n");
    }
    double object_pose[7] = {0.0};

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            const mjtNum* position = d->xpos + 3 * body_id;   // xpos는 3 * body_id 크기 배열
            const mjtNum* orientation = d->xquat + 4 * body_id; // xquat는 4 * body_id 크기 배열

              // 위치와 방향 출력
              // printf("Position of object_box1: [%f, %f, %f]\n", position[0], position[1], position[2]);
              // printf("Orientation (quaternion) of object_box1: [%f, %f, %f, %f]\n",
              //       orientation[0], orientation[1], orientation[2], orientation[3]);

            TaskController.read(d->time, d->qpos, d->qvel);
            // TaskController.read(d->time, d->qpos, d->qvel, object_pose); // controller_j15
            TaskController.control_mujoco();
            TaskController.write(d->ctrl);
            // visualize_state_planner(sim);

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }
              const mjtNum* position = d->xpos + 3 * body_id;   // xpos는 3 * body_id 크기 배열
              const mjtNum* orientation = d->xquat + 4 * body_id; // xquat는 4 * body_id 크기 배열
              
              
              for(int i=0; i<3; i++)
              {
                object_pose[i] = position[i];
              }
              for(int i=0; i<4; i++)
              {
                object_pose[i+3] = orientation[i];
              }

              TaskController.read(d->time, d->qpos, d->qvel);
              // TaskController.read(d->time, d->qpos, d->qvel, object_pose); // controller_j15
              TaskController.control_mujoco();
              TaskController.write(d->ctrl);
              visualize_state_planner(sim);

              // inject noise
              sim.InjectNoise();

              // call mj_step
              mj_step(m, d);
              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  const char* filename = nullptr;
  if (argc >  1) {
    filename = argv[1];
  }

  char str[100] = "../model/dualarm_hands_mod_sim.xml";
  filename = str;

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
