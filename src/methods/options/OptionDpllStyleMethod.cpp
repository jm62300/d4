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

#include "OptionDpllStyleMethod.hpp"

#include "OptionOperationManager.hpp"

namespace d4 {

/**
 * @brief OptionDpllStyleMethod::OptionDpllStyleMethod implementation.
 */
OptionDpllStyleMethod::OptionDpllStyleMethod(
    const ConfigurationDpllStyleMethod& config) {
  // Operator used.
  optionOperationManager.operatorType = config.operationType;

  // Cache Options:
  optionCacheManager.cachingMethod = config.cache.cachingMethod;

  optionCacheManager.optionCacheCleaningManager = {
      config.cache.cacheCleaningStrategy};

  optionCacheManager.optionBucketManager = {
      config.cache.modeStore,     config.cache.clauseRepresentation,
      config.cache.sizeFirstPage, config.cache.sizeAdditionalPage,
      config.cache.limitVarSym,   config.cache.limitVarIndex};

  optionSolver = {config.solver.solverName};

  optionSpecManager = {config.spec.specUpdateType};

  optionBranchingHeuristic = {config.branchingHeuristic.scoringMethodType,
                              config.branchingHeuristic.phaseHeuristicType,
                              config.branchingHeuristic.reversePhase};

  optionPartitioningHeuristic = {
      config.partitioningHeuristic.partitioningMethod,
      config.partitioningHeuristic.partitionerName,
      config.partitioningHeuristic.reduceFormula,
      config.partitioningHeuristic.equivSimp,
      config.partitioningHeuristic.staticPhase,
      config.partitioningHeuristic.dynamicPhase};

  freqDecay = config.freqDecay;
  cacheIsActivated = config.cache.isActivated;
}  // constructor

}  // namespace d4