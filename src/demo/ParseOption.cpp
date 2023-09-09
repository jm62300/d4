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

#include "ParseOption.hpp"

/**
 * @brief parsePreprocConfiguration implementation.
 */
d4::ConfigurationPeproc parsePreprocConfiguration(po::variables_map &vm) {
  d4::ConfigurationPeproc config;
  config.inputType =
      d4::InputTypeManager::getInputType(vm["input-type"].as<std::string>());
  config.nbIteration = vm["preproc-reducer-iteration"].as<int>();
  config.preprocMethod = d4::PreprocMethodManager::getPreprocMethod(
      vm["preproc"].as<std::string>());
  config.timeout = vm["preproc-timeout"].as<int>();

  return config;
}  // parsePreprocConfiguration

/**
 * @brief parseCacheConfiguration implementation.
 */
d4::ConfigurationCache parseCacheConfiguration(po::variables_map &vm) {
  d4::ConfigurationCache cache;

  cache.cachingMethod = d4::CachingMehodManager::getCachingMethod(
      vm["cache-method"].as<std::string>());

  cache.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy(
          vm["cache-reduction-strategy"].as<std::string>());

  cache.modeStore = d4::ModeStoreManager::getModeStore(
      vm["cache-store-strategy"].as<std::string>());

  cache.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation(
          vm["cache-clause-representation"].as<std::string>());

  cache.sizeFirstPage = vm["cache-size-first-page"].as<unsigned long>();

  cache.sizeAdditionalPage =
      vm["cache-size-additional-page"].as<unsigned long>();

  cache.limitVarSym =
      vm["cache-clause-representation-combi-limitVar-sym"].as<unsigned>();

  cache.isActivated = vm["cache-activated"].as<bool>();

  cache.limitVarIndex =
      vm["cache-clause-representation-combi-limitVar-index"].as<unsigned>();

  return cache;
}  // parseCacheConfiguration