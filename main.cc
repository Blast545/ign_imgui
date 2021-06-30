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

#include "CsvUtils.hh"
#include "Histogram.hh"

using namespace ignition;

const size_t kDefaultHistBins = 100;
const float kDefaultHistMin = 0.0f;
const float kDefaultHistMax = 2.0f;

const float kDefaultRTFMin = 0.0f;
const float kDefaultRTFMax = 2.0f;

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

bool shouldClose{false};

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
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

  float rtfMin = kDefaultRTFMin;
  float rtfMax = kDefaultRTFMax;

  while(!shouldClose)
  {
    {
      // Callback from /clock topic is saving all the info
      //std::lock_guard<std::mutex> lock(rtfsMutex);
      //ignition::common::Time real_z(msg_z.real().sec(), msg_z.real().nsec());
      //ignition::common::Time sim_z(msg_z.sim().sec(), msg_z.sim().nsec());
    }
  }
  node.Unsubscribe("/clock");

  if (outputCsv.size()) {
    std::ofstream fs;
    fs.open(outputCsv, std::ios::trunc);
    ign_imgui::ToCsv(fs, stats, hist, sim_z.Double(), real_z.Double());
    fs.close();
  }

  return 0;
}
