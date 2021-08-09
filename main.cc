/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <cmath>
#include <csignal>

#include <ignition/msgs.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/SignalStats.hh>
#include <ignition/transport/Node.hh>

#include <imgui/imgui.h>
#include <imgui/examples/imgui_impl_glfw.h>
#include <imgui/examples/imgui_impl_opengl3.h>

#include "CsvUtils.hh"
#include "Histogram.hh"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace ignition;

const size_t kDefaultHistBins = 100;
const float kDefaultHistMin = 0.0f;
const float kDefaultHistMax = 2.0f;

const float kDefaultRTFMin = 0.0f;
const float kDefaultRTFMax = 2.0f;

//////////////////////////////////////////////////
static void glfw_error_callback(int error, const char* description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

namespace ign_imgui
{

//////////////////////////////////////////////////
void ToCsv(
  std::ostream & ost, const ignition::math::SignalStats & stats,
  const ign_imgui::Histogram & hist, double simTime, double realTime)
{
  ost << simTime << "," << realTime << "," << std::endl;
  ost << stats.Count() << "," << stats.Map()["mean"] << "," << stats.Map()["var"] <<
    "," << stats.Map()["min"] << "," << stats.Map()["max"] << "," << std::endl;
  hist.ToCsv(ost);
}

struct LoadedData
{
  size_t count;
  double mean;
  double var;
  double max;
  double min;

  double realTime;
  double simTime;
};

//////////////////////////////////////////////////
LoadedData FromCsv(
  std::istream & ist, ign_imgui::Histogram & hist)
{
  using ign_imgui::GetNextCsv;
  using ign_imgui::GetNewLine;

  LoadedData data;
  GetNextCsv(ist, data.simTime);
  GetNextCsv(ist, data.realTime);
  GetNewLine(ist);
  GetNextCsv(ist, data.count);
  GetNextCsv(ist, data.mean);
  GetNextCsv(ist, data.var);
  GetNextCsv(ist, data.min);
  GetNextCsv(ist, data.max);
  GetNewLine(ist);

  hist.FromCsv(ist);
  while (ist.good()) {
    std::string str;
    ist >> str;
    std::cout << str;
  }
  std::cout << std::endl;
  IGN_IMGUI_CHECK_STREAM(ist, eof);

  return data;
}

}  // namespace ign_imgui

std::atomic<bool> shouldClose{false};

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  // Initialize OpenGL
  glfwSetErrorCallback(glfw_error_callback);
  if(!glfwInit())
    return 1;

  std::signal(SIGINT, [](int) {
    shouldClose = true;
  });

  std::string outputCsv;
  std::string inputCsv;
  for (size_t i = 1; i < _argc; ++i) {
    if (i + 1u < _argc) {
      if (0 == strcmp(_argv[i], "--output") || 0 == strcmp(_argv[i], "-o")) {
        outputCsv = _argv[++i];
        ++i;
        continue;
      }
      if (0 == strcmp(_argv[i], "--input") || 0 == strcmp(_argv[i], "-i")) {
        inputCsv = _argv[++i];
        ++i;
        continue;
      }
    }
    std::cout << std::endl << _argv[0] << " [--output <OUTPUT_FILE_PATH>] [--input <OUTPUT_FILE_PATH>]" << std::endl;
    std::exit(0);
  }

  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  GLFWwindow* window = glfwCreateWindow(400, 400, "ign_imgui", NULL, NULL);
  if (window == NULL)
    return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  bool err = glewInit() != 0;

  // Set verbosity
  ignition::common::Console::SetVerbosity(4);
  ignition::transport::Node node;

  std::mutex rtfsMutex;
  ignition::msgs::Clock msg_z;
  bool first = true;

  bool animate = true;

  std::vector<float> rtfs;

  ignition::math::SignalStats stats;
  stats.InsertStatistic("max");
  stats.InsertStatistic("min");
  stats.InsertStatistic("mean");
  stats.InsertStatistic("var");

  ign_imgui::Histogram hist;

  hist.SetNumBins(200);
  hist.SetRange(0.0f, 2.0f);

  ignition::common::Time real_z{};
  ignition::common::Time sim_z{};

  bool usingLoadedData{false};
  ign_imgui::LoadedData loadedData;


  if (inputCsv.size()) {
    std::ifstream fs;
    fs.open(inputCsv);
    loadedData = ign_imgui::FromCsv(fs, hist);
    usingLoadedData = true;
  }

  if (!usingLoadedData) {
    std::function<void(const ignition::msgs::Clock&)> cb =
      [&](const ignition::msgs::Clock &_msg)
      {
        std::lock_guard<std::mutex> lock(rtfsMutex);

        if (first)
        {
          msg_z = _msg;
          first = false;
          return;
        }

        real_z = ignition::common::Time(msg_z.real().sec(), msg_z.real().nsec());
        sim_z= ignition::common::Time(msg_z.sim().sec(), msg_z.sim().nsec());
        ignition::common::Time real(_msg.real().sec(), _msg.real().nsec());
        ignition::common::Time sim(_msg.sim().sec(), _msg.sim().nsec());

        auto real_dt = (real - real_z);
        auto sim_dt = (sim - sim_z);
        auto rtf = sim_dt.Double() / real_dt.Double();

        msg_z = _msg;

        if (animate && std::isfinite(rtf))
        {
          stats.InsertData(rtf);
          hist.InsertData(rtf);

          if (rtfs.size() > 250)
          {
            for(size_t ii = 1; ii < rtfs.size(); ++ii)
            {
              rtfs[ii-1] = rtfs[ii];
            }
            rtfs[rtfs.size() - 1] = rtf;
          }
          else
          {
            rtfs.push_back(rtf);
          }
        }

      };
    node.Subscribe("/clock", cb);
  }
  double progress = 0;

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  float rtfMin = kDefaultRTFMin;
  float rtfMax = kDefaultRTFMax;

  while (!glfwWindowShouldClose(window) && !shouldClose)
  {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    //ImGui::ShowDemoWindow();

    {
      std::lock_guard<std::mutex> lock(rtfsMutex);

      bool p_open;
      ImGui::Begin("RTF", &p_open, ImGuiWindowFlags_AlwaysAutoResize);

      ImGui::Checkbox("Animate", &animate);

      // Line plot
      ImGui::PlotLines("RTF", &rtfs[0], rtfs.size(), 0, NULL, rtfMin, rtfMax,
          ImVec2(400, 400));

      ImGui::InputFloat("RTF Y-axis min", &rtfMin, 0.0f, 10.0f, "%.3f");
      ImGui::InputFloat("RTF Y-axis max", &rtfMax, 0.0f, 10.0f, "%.3f");

      // Histogram
      ImGui::Separator();
      hist.PlotHistogram("RTF Histogram", ImVec2(400, 400));

      if (ImGui::Button("Reset"))
      {
        hist.Reset();
        stats.Reset();
        usingLoadedData = false;
      }

      // Statistics
      ImGui::Separator();
      if (!usingLoadedData) {
        ImGui::Text("Samples: %zi", stats.Count());
        ImGui::Text("Mean: %f", stats.Map()["mean"]);
        ImGui::Text("Var: %f", stats.Map()["var"]);
        ImGui::Text("Max: %f", stats.Map()["max"]);
        ImGui::Text("Min: %f", stats.Map()["min"]);
      } else {
        ImGui::Text("Samples: %zi", loadedData.count);
        ImGui::Text("Mean: %f", loadedData.mean);
        ImGui::Text("Var: %f", loadedData.var);
        ImGui::Text("Max: %f", loadedData.max);
        ImGui::Text("Min: %f", loadedData.min);
      }

      ImGui::Separator();

      ignition::common::Time real_z(msg_z.real().sec(), msg_z.real().nsec());
      ignition::common::Time sim_z(msg_z.sim().sec(), msg_z.sim().nsec());

      if (!usingLoadedData) {
        ImGui::Text("Real Time: %.3f", real_z.Double());
        ImGui::Text("Sim Time: %.3f", sim_z.Double());
        ImGui::Text("Elapsed RTF: %.3f", sim_z.Double() / real_z.Double());
      } else {
        ImGui::Text("Real Time: %.3f", loadedData.realTime);
        ImGui::Text("Sim Time: %.3f", loadedData.simTime);
        ImGui::Text("Elapsed RTF: %.3f", loadedData.simTime / loadedData.realTime);
      }


      ImGui::End();
    }

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  node.Unsubscribe("/clock");

  if (outputCsv.size()) {
    std::ofstream fs;
    fs.open(outputCsv, std::ios::trunc);
    ign_imgui::ToCsv(fs, stats, hist, sim_z.Double(), real_z.Double());
    fs.close();
  }

  return 0;
}
