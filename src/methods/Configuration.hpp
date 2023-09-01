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

#include <string>

#include "src/caching/OptionCacheManager.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/solvers/OptionSolver.hpp"
#include "src/specs/OptionSpecManager.hpp"

namespace d4 {
enum MethodName {
  METH_EROSION,
  METH_COUNTING,
  METH_DDNNF,
  METH_ERE,
  METH_MAX_SHARP,
  METH_MIN_SHARP,
  METH_PROJ_MC,
  METH_NONE
};

class MethodNameManager {
 public:
  static std::string getMethodName(const MethodName& m) {
    if (m == METH_EROSION) return "erosion";
    if (m == METH_COUNTING) return "counting";
    if (m == METH_DDNNF) return "ddnnf-compiler";
    if (m == METH_MAX_SHARP) return "max#sat";
    if (m == METH_MIN_SHARP) return "min#sat";
    if (m == METH_ERE) return "ere";
    if (m == METH_PROJ_MC) return "projMC";
    if (m == METH_NONE) return "none";

    throw(FactoryException("Method name unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static MethodName getMethodName(const std::string& m) {
    if (m == "erosion") return METH_EROSION;
    if (m == "counting") return METH_COUNTING;
    if (m == "ddnnf-compiler") return METH_DDNNF;
    if (m == "max#sat") return METH_MAX_SHARP;
    if (m == "min#sat") return METH_MIN_SHARP;
    if (m == "ere") return METH_ERE;
    if (m == "projMC") return METH_PROJ_MC;
    if (m == "none") return METH_NONE;

    throw(FactoryException("Method name unknown", __FILE__, __LINE__));
  }  // getOperatorType
};

class Configuration {
 public:
  MethodName methodName;

  struct ConfigurationDpllSyleMethod {
    unsigned freqDecay;

    struct ConfigurationCache {
      CachingMethod cachingMethod;
      CacheCleaningStrategy cacheCleaningStrategy;
      ModeStore modeStore;
      ClauseRepresentation clauseRepresentation;

      bool isActivated;
      unsigned long sizeFirstPage;
      unsigned long sizeAdditionalPage;
      unsigned limitVarSym;
      unsigned limitVarIndex;
    } cache;

    struct ConfigurationSolver {
      SolverName solverName;
    } solver;

    struct ConfigurationSpec {
      SpecUpdateType specUpdateType;
    } spec;

  } dpllConfig;

  friend std::ostream& operator<<(std::ostream& out, const Configuration& dt) {
    out << "c Configuration:\n"
        << "c Method used:" << MethodNameManager::getMethodName(dt.methodName)
        << "\n";

    return out;
  }  // <<
};
}  // namespace d4