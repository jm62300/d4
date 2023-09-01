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

namespace d4 {

/**
 * @brief OptionDpllStyleMethod::OptionDpllStyleMethod implementation.
 */
OptionDpllStyleMethod::OptionDpllStyleMethod(const Configuration& config) {
  // Operator used.
  optionOperationManager.operatorType =
      OperationTypeManager::getOperatorType(config.methodName);

  // Cache Options:
  optionCacheManager.cachingMethod = config.dpllConfig.cache.cachingMethod;

  optionCacheManager.optionCacheCleaningManager = {
      config.dpllConfig.cache.cacheCleaningStrategy};

  optionCacheManager.optionBucketManager = {
      config.dpllConfig.cache.modeStore,
      config.dpllConfig.cache.clauseRepresentation,
      config.dpllConfig.cache.sizeFirstPage,
      config.dpllConfig.cache.sizeAdditionalPage,
      config.dpllConfig.cache.limitVarSym,
      config.dpllConfig.cache.limitVarIndex};

  optionSolver = {config.dpllConfig.solver.solverName};

  freqDecay = config.dpllConfig.freqDecay;
  cacheIsActivated = config.dpllConfig.cache.isActivated;
}  // constructor

}  // namespace d4