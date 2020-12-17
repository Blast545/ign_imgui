/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGN_IMGUI__CSV_UTILS_HH_
#define IGN_IMGUI__CSV_UTILS_HH_

#include <istream>
#include <sstream>
#include <stdexcept>
#include <string>

#define IGN_IMGUI_CHECK_STREAM(stream, condition) \
  if (!stream.condition()) { \
    throw std::runtime_error{"failed to parse input csv file"}; \
  }

namespace ign_imgui
{

template<typename T>
void GetNextCsv(std::istream & ist, T & value)
{
    std::string str;
    std::getline(ist, str, ',');
    std::istringstream sst(str);
    IGN_IMGUI_CHECK_STREAM(ist, good);
    sst >> value;
    IGN_IMGUI_CHECK_STREAM(sst, eof);
}

inline void GetNewLine(std::istream & ist) {
  ist >> std::ws;
  IGN_IMGUI_CHECK_STREAM(ist, good);
}

}  // namespace ign_imgui

#endif  // IGN_IMGUI__CSV_UTILS_HH_

