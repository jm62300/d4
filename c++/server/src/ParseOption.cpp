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
 * @brief parseCacheConfiguration implementation.
 */
d4::OptionCacheManager parseCacheConfiguration(const po::variables_map &vm,
                                               const std::string &prefix) {
  d4::OptionCacheManager cache;

  cache.cachingMethod = d4::CachingMethodManager::getCachingMethod(
      vm[prefix + "cache-method"].as<std::string>());

  cache.optionCacheCleaningManager.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy(
          vm[prefix + "cache-reduction-strategy"].as<std::string>());

  cache.optionBucketManager.modeStore = d4::ModeStoreManager::getModeStore(
      vm[prefix + "cache-store-strategy"].as<std::string>());

  cache.optionBucketManager.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation(
          vm[prefix + "cache-clause-representation"].as<std::string>());

  cache.optionBucketManager.sizeFirstPage =
      vm[prefix + "cache-size-first-page"].as<unsigned long>();

  cache.optionBucketManager.sizeAdditionalPage =
      vm[prefix + "cache-size-additional-page"].as<unsigned long>();

  cache.optionBucketManager.limitVarSym =
      vm[prefix + "cache-clause-representation-combi-limitVar-sym"]
          .as<unsigned>();

  cache.isActivated = vm[prefix + "cache-activated"].as<bool>();

  cache.optionBucketManager.limitVarIndex =
      vm[prefix + "cache-clause-representation-combi-limitVar-index"]
          .as<unsigned>();

  return cache;
}  // parseCacheConfiguration

/**
 * @brief parseBranchingHeuristicConfiguration implementation.
 */
d4::OptionBranchingHeuristic parseBranchingHeuristicConfiguration(
    const po::variables_map &vm, const std::string &prefix) {
  d4::OptionBranchingHeuristic branchingHeuristic;

  branchingHeuristic.freqDecay =
      vm[prefix + "scoring-method-freq-decay"].as<unsigned>();

  branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType(
          vm[prefix + "scoring-method"].as<std::string>());

  branchingHeuristic.branchingHeuristicType =
      d4::BranchingHeuristicTypeManager::getBranchingHeuristicType(
          vm[prefix + "branching-heuristic"].as<std::string>());

  branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType(
          vm[prefix + "phase-heuristic"].as<std::string>());

  branchingHeuristic.reversePhase =
      vm[prefix + "phase-heuristic-reversed"].as<bool>();

  branchingHeuristic.limitSizeClause =
      vm[prefix + "branching-heuristic-limit-clause"].as<unsigned>();

  return branchingHeuristic;
}  // parseBranchingHeuristicConfiguration

/**
 * @brief parsePartitioningHeuristicConfiguration implementation.
 */
d4::OptionPartialOrderHeuristic parsePartitioningHeuristicConfiguration(
    const po::variables_map &vm, const std::string &prefix) {
  d4::OptionPartialOrderHeuristic partitioningHeuristic;
  partitioningHeuristic.partialOrderMethod =
      d4::PartialOrderMethodManager::getPartialOrderMethod(
          vm[prefix + "partitioning-heuristic"].as<std::string>());

  partitioningHeuristic.partitionerName =
      d4::PartitionerNameManager::getPartitionerName(
          vm[prefix + "partitioning-heuristic-partitioner"].as<std::string>());

  return partitioningHeuristic;
}  // parsePartitioningHeuristicConfiguration