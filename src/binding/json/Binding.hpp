#pragma once

#include <nlohmann/json.hpp>
#include <map>
#include <string>

#include "src/configurations/Configuration.hpp"
#include "src/configurations/ConfigurationBranchingHeuristic.hpp"
#include "src/configurations/ConfigurationCache.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/configurations/ConfigurationEREMethod.hpp"
#include "src/configurations/ConfigurationFormulaManager.hpp"
#include "src/configurations/ConfigurationMaxSharpSatMethod.hpp"
#include "src/configurations/ConfigurationMaxTMethod.hpp"
#include "src/configurations/ConfigurationMinSharpSatMethod.hpp"
#include "src/configurations/ConfigurationPartialOrderHeuristic.hpp"
#include "src/configurations/ConfigurationPreproc.hpp"
#include "src/configurations/ConfigurationProjMcMethod.hpp"
#include "src/configurations/ConfigurationQbfCounter.hpp"
#include "src/configurations/ConfigurationSolver.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/options/cache/OptionCacheCleaningManager.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/methods/OptionEREMethod.hpp"
#include "src/options/methods/OptionMaxSharpSatMethod.hpp"
#include "src/options/methods/OptionMaxTMethod.hpp"
#include "src/options/methods/OptionMethodManager.hpp"
#include "src/options/methods/OptionMinSharpSatMethod.hpp"
#include "src/options/methods/OptionOperationManager.hpp"
#include "src/options/methods/OptionProjMcMethod.hpp"
#include "src/options/methods/OptionQbfCounter.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"
#include "src/options/solvers/OptionSolver.hpp"

namespace d4 {
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationPeproc, preprocMethod, inputType, nbIteration, ordered, onlyUseGates, strongElim, timeout);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Configuration, methodName, precision, isFloat, inputName, problemInputType, configurationPreproc);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationQbfCounter, cache, solver, spec, branchingHeuristic, partitioningHeuristic);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationMaxSharpSatMathod, greedyInitActivated, digOnAnd, threshold, solver, specManager, phaseHeuristicMax, randomPhaseHeuristicMax, branchingHeuristicMax, cacheManagerMax, branchingHeuristicInd, cacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationMinSharpSatMathod, greedyInitActivated, digOnAnd, threshold, solver, specManager, phaseHeuristicMin, randomPhaseHeuristicMin, branchingHeuristicMin, cacheManagerMin, branchingHeuristicInd, cacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationPartialOrderHeuristic, partialOrderMethod, partitionerName, treeDecompositionMethod, treeDecompositionerMethod, hyperGraphExtractorMethod, graphExtractorMethod, useSimpGraphExtractor, budget, seed, verbosity, givenOrder, scaleFactor);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationBranchingHeuristic, configurationPartialOrderHeuristic, scoringMethodType, phaseHeuristicType, branchingHeuristicType, reversePhase, freqDecay, limitSizeClause);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationEREMethod, greedyInitActivated, digOnAnd, threshold, solver, specManager, cutExist, randomPhaseHeuristicExist, phaseHeuristicBestExist, branchingHeuristicExist, cacheManagerExist, branchingHeuristicRandom, cacheManagerRandom, computeComponentOnRandom);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationProjMcMethod, refinement, cache, solver, specs, counter);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationMaxTMethod, greedyInitActivated, solver, specManager, phaseHeuristicMax, randomPhaseHeuristicMax, branchingHeuristicMax, cacheManagerMax, branchingHeuristicInd, cacheManagerInd, thresholdList);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationDpllStyleMethod, operationType, cache, solver, spec, branchingHeuristic, exploitModel, verbosity);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationSolver, solverName);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationCache, cachingMethod, cacheCleaningStrategy, modeStore, clauseRepresentation, isActivated, sizeFirstPage, sizeAdditionalPage, limitVarSym, limitVarIndex);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationSpec, specUpdateType, removeGates, needFastNotSatisfied);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionSolver, solverName);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionBucketManager, modeStore, clauseRepresentation, sizeFirstPage, sizeAdditionalPage, limitNbVarSym, limitNbVarIndex);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionCacheManager, cachingMethod, optionBucketManager, optionCacheCleaningManager, isActivated);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionCacheCleaningManager, cacheCleaningStrategy);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionBranchingHeuristic, optionPartialOrderHeuristic, scoringMethodType, phaseHeuristicType, branchingHeuristicType, reversePhase, freqDecay, limitSizeClause);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionSpecManager, specUpdateType, removeGates, needFastNotSatisfied);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionPreprocManager, inputType, preprocMethod, nbIteration, onlyUseGates, ordered, strongElim, timeout);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionOperationManager, operatorType);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMaxSharpSatMethod, greedyInitActivated, digOnAnd, threshold, optionSolver, optionSpecManager, phaseHeuristicMax, randomPhaseHeuristicMax, optionBranchingHeuristicMax, optionCacheManagerMax, optionBranchingHeuristicInd, optionCacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMethodManager, optionOperationManager);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMaxTMethod, greedyInitActivated, thresholdList, optionSolver, optionSpecManager, phaseHeuristicMax, randomPhaseHeuristicMax, optionBranchingHeuristicMax, optionCacheManagerMax, optionBranchingHeuristicInd, optionCacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionQbfCounter, optionCacheManager, optionSolver, optionSpecManager, optionBranchingHeuristic);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionProjMcMethod, refinement, optionCache, optionSolver, optionSpecs, optionCounter);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionDpllStyleMethod, optionOperationManager, optionCacheManager, optionSolver, optionSpecManager, optionBranchingHeuristic, exploitModel, verbosity);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMinSharpSatMethod, greedyInitActivated, digOnAnd, threshold, optionSolver, optionSpecManager, phaseHeuristicMin, randomPhaseHeuristicMin, optionBranchingHeuristicMin, optionCacheManagerMin, optionBranchingHeuristicInd, optionCacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionEREMethod, greedyInitActivated, digOnAnd, threshold, optionSolver, optionSpecManager, cutExist, phaseHeuristicBestExist, randomPhaseHeuristicExist, optionBranchingHeuristicExist, optionCacheManagerExist, computeComponentOnRandom, optionBranchingHeuristicRandom, optionCacheManagerRandom);

    template<typename ConfigStruct>
    ConfigStruct from_string_map(const std::map<std::string, std::string>& string_map) {
        nlohmann::json patch_j;

        for (const auto& [key, value_str] : string_map) {
            
            std::string json_ptr_path = "/" + key;
            std::replace(json_ptr_path.begin(), json_ptr_path.end(), '.', '/');

            try {

                patch_j[nlohmann::json::json_pointer(json_ptr_path)] = nlohmann::json::parse(value_str);
            } 
            catch (const nlohmann::json::parse_error&) {
                patch_j[nlohmann::json::json_pointer(json_ptr_path)] = value_str;
            }
        }

        ConfigStruct default_obj;
        nlohmann::json full_j = default_obj;
        full_j.merge_patch(patch_j);

        return full_j.get<ConfigStruct>();
    }

    template<typename ConfigStruct>
    ConfigStruct from_json_string(const std::string& json_str) {
        nlohmann::json input_j = nlohmann::json::parse(json_str);

        ConfigStruct default_obj;
        nlohmann::json full_j = default_obj;
        full_j.merge_patch(input_j);

        return full_j.get<ConfigStruct>();
    }
} // namespace d4
