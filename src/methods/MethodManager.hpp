/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once

#include "src/problem/ProblemManager.hpp"
#include "src/options/EnumMetadata.hpp"

namespace d4 {

enum MethodName {
  METH_EROSION,
  METH_COUNTING,
  METH_DDNNF,
  METH_ERE,
  METH_MAX_SHARP,
  METH_MIN_SHARP,
  METH_PROJ_MC,
  METH_COUNTING_GLOBAL_CACHE,
  METH_QBF_COUNTER,
  METH_NONE
};

template <>
struct EnumMetadata<MethodName> {
  static std::string name() { return "MethodName"; }
  static std::map<int, std::string> mapping() {
    return {
        {METH_EROSION, "erosion"},
        {METH_COUNTING, "counting"},
        {METH_DDNNF, "ddnnf-compiler"},
        {METH_MAX_SHARP, "max#sat"},
        {METH_MIN_SHARP, "min#sat"},
        {METH_ERE, "ere"},
        {METH_PROJ_MC, "projMC"},
        {METH_COUNTING_GLOBAL_CACHE, "counting-global-cache"},
        {METH_QBF_COUNTER, "qbf-counter"},
        {METH_NONE, "none"}};
  }
};

class MethodManager {
 protected:
  std::clock_t currentTime;

 public:
  virtual ~MethodManager() {}

  static void displayInfoVariables(const ProblemManager& problem,
                                   std::ostream& out);

  virtual void interrupt() {}

  inline void initTimer() { currentTime = clock(); }
  inline float getTimer() {
    return (float)(clock() - currentTime) / CLOCKS_PER_SEC;
  }
};
}  // namespace d4
