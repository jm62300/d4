#pragma once

#include <nlohmann/json.hpp>
#include <map>
#include <stdexcept>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/methods/OptionEREMethod.hpp"
#include "src/options/methods/OptionMaxSharpSatMethod.hpp"
#include "src/options/methods/OptionMaxTMethod.hpp"
#include "src/options/methods/OptionMethodManager.hpp"
#include "src/options/methods/OptionMinSharpSatMethod.hpp"
#include "src/options/methods/OptionProjMcMethod.hpp"
#include "src/options/methods/OptionQbfCounter.hpp"
#include "src/options/solvers/OptionSolver.hpp"

#include "src/options/OptionRegistry.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/treeDecomposition/TreeDecomposition.hpp"
#include "src/treeDecompositioner/TreeDecompositioner.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"
#include "src/representation/graph/GraphExtractor.hpp"

namespace d4 {
    template<typename E>
    typename std::enable_if<std::is_enum<E>::value, void>::type
    to_json(nlohmann::json& j, const E& e) {
        auto m = EnumMetadata<E>::mapping();
        auto it = m.find(static_cast<int>(e));
        if (it != m.end()) {
            j = it->second;
        } else {
            j = static_cast<int>(e);
        }
    }

    template<typename E>
    typename std::enable_if<std::is_enum<E>::value, void>::type
    from_json(const nlohmann::json& j, E& e) {
        if (j.is_number()) {
            e = static_cast<E>(j.get<int>());
            return;
        }
        std::string s = j.get<std::string>();
        auto m = EnumMetadata<E>::mapping();
        for (auto const& [val, label] : m) {
            if (label == s) {
                e = static_cast<E>(val);
                return;
            }
        }
        // Fallback to integer conversion if string match fails
        try {
            e = static_cast<E>(std::stoi(s));
        } catch (...) {
            throw std::runtime_error("Unknown enum value: " + s);
        }
    }

    // --- Option Template JSON Support ---
    template<typename T>
    void to_json(nlohmann::json& j, const Option<T>& opt) {
        j = opt.get();
    }
    template<typename T>
    void from_json(const nlohmann::json& j, Option<T>& opt) {
        opt.set(j.get<T>());
    }

    inline void update_registry_recursive(OptionRegistry& registry, const nlohmann::json& j, const std::string& prefix) {
        for (auto it = j.begin(); it != j.end(); ++it) {
            std::string key = prefix + it.key();
            if (it->is_object()) {
                update_registry_recursive(registry, *it, key + ".");
            } else {
                if (registry.hasOption(key)) {
                    if (it->is_string()) {
                        registry.setOption(key, it->get<std::string>());
                    } else {
                        registry.setOption(key, it->dump());
                    }
                }
            }
        }
    }

    inline void parse_json_to_registry(OptionRegistry& registry, const std::string& json_str) {
        try {
            auto j = nlohmann::json::parse(json_str);
            update_registry_recursive(registry, j, "");
        } catch (...) {
            // Ignore parse errors for now
        }
    }

    inline void update_registry_from_json(OptionRegistry& registry, const std::string& json_str) {
        parse_json_to_registry(registry, json_str);
    }

    inline nlohmann::json convert_registry_to_json(const OptionRegistry& registry) {
        nlohmann::json root = nlohmann::json::object();
        for (auto const& [path, opt] : registry.getOptions()) {
            std::string json_ptr_path = "/" + path;
            std::replace(json_ptr_path.begin(), json_ptr_path.end(), '.', '/');
            
            std::string val = opt->getValueAsString();
            std::string type = opt->getTypeName();

            try {
                if (type == "bool") {
                    root[nlohmann::json::json_pointer(json_ptr_path)] = (val == "true");
                } else if (type == "int" || type == "uint") {
                    root[nlohmann::json::json_pointer(json_ptr_path)] = std::stoll(val);
                } else if (type == "float" || type == "double") {
                    root[nlohmann::json::json_pointer(json_ptr_path)] = std::stod(val);
                } else {
                    root[nlohmann::json::json_pointer(json_ptr_path)] = val;
                }
            } catch (...) {
                root[nlohmann::json::json_pointer(json_ptr_path)] = val;
            }
        }
        return root;
    }

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionRoot, methodName, precision, isFloat, inputName, problemInputType);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionQbfCounter, methodName, precision, isFloat, inputName, problemInputType, optionCacheManager, optionSolver, optionSpecManager, optionBranchingHeuristic, optionPartitioningHeuristic);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMaxSharpSatMethod, methodName, precision, isFloat, inputName, problemInputType, greedyInitActivated, digOnAnd, threshold, optionSolver, optionSpecManager, phaseHeuristicMax, randomPhaseHeuristicMax, optionBranchingHeuristicMax, optionCacheManagerMax, optionBranchingHeuristicInd, optionCacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMinSharpSatMethod, methodName, precision, isFloat, inputName, problemInputType, greedyInitActivated, digOnAnd, threshold, optionSolver, optionSpecManager, phaseHeuristicMin, randomPhaseHeuristicMin, optionBranchingHeuristicMin, optionCacheManagerMin, optionBranchingHeuristicInd, optionCacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionPartialOrderHeuristic, partialOrderMethod, partitionerName, treeDecompositionMethod, treeDecompositionerMethod, hyperGraphExtractorMethod, graphExtractorMethod, useSimpGraphExtractor, budget, seed, verbosity, givenOrder, scaleFactor);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionBranchingHeuristic, optionPartialOrderHeuristic, scoringMethodType, phaseHeuristicType, branchingHeuristicType, reversePhase, freqDecay, limitSizeClause);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionEREMethod, methodName, precision, isFloat, inputName, problemInputType, greedyInitActivated, digOnAnd, threshold, optionSolver, optionSpecManager, cutExist, phaseHeuristicBestExist, randomPhaseHeuristicExist, optionBranchingHeuristicExist, optionCacheManagerExist, computeComponentOnRandom, optionBranchingHeuristicRandom, optionCacheManagerRandom);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionProjMcMethod, methodName, precision, isFloat, inputName, problemInputType, refinement, optionCache, optionSolver, optionSpecs, optionCounter);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMaxTMethod, methodName, precision, isFloat, inputName, problemInputType, greedyInitActivated, thresholdList, optionSolver, optionSpecManager, phaseHeuristicMax, randomPhaseHeuristicMax, optionBranchingHeuristicMax, optionCacheManagerMax, optionBranchingHeuristicInd, optionCacheManagerInd);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionDpllStyleMethod, methodName, precision, isFloat, inputName, problemInputType, operationType, optionCacheManager, optionSolver, optionSpecManager, optionBranchingHeuristic, exploitModel, verbosity);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionSolver, solverName);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionCacheCleaningManager, cacheCleaningStrategy);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionBucketManager, modeStore, clauseRepresentation, sizeFirstPage, sizeAdditionalPage, limitVarSym, limitVarIndex);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionCacheManager, cachingMethod, optionBucketManager, optionCacheCleaningManager, isActivated);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionSpecManager, specUpdateType, removeGates, needFastNotSatisfied);
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OptionMethodManager, operationType);

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

    // -------------------------------------------------------------------------
    // Internal helper: parse argv into a JSON patch object.
    // Only tokens starting with "--" are processed.
    // -------------------------------------------------------------------------
    inline nlohmann::json argv_to_json_patch(int argc, char* argv[]) {
        nlohmann::json patch = nlohmann::json::object();

        for (int i = 0; i < argc; ++i) {
            std::string arg(argv[i]);

            if (arg.size() < 3 || arg[0] != '-' || arg[1] != '-')
                continue;

            std::string token = arg.substr(2);
            std::string key, value;

            const auto eq_pos = token.find('=');
            if (eq_pos != std::string::npos) {
                // Form: --key=value
                key   = token.substr(0, eq_pos);
                value = token.substr(eq_pos + 1);
            } else if (i + 1 < argc) {
                // Form: --key value
                std::string next(argv[i + 1]);
                if (next.size() < 2 || next[0] != '-' || next[1] != '-') {
                    key   = token;
                    value = next;
                    ++i;
                } else {
                    key   = token;
                    value = "true"; // Boolean flag
                }
            } else {
                key   = token;
                value = "true"; // Boolean flag
            }

            std::string json_ptr_path = "/" + key;
            std::replace(json_ptr_path.begin(), json_ptr_path.end(), '.', '/');

            try {
                patch[nlohmann::json::json_pointer(json_ptr_path)] =
                    nlohmann::json::parse(value);
            } catch (const nlohmann::json::parse_error&) {
                patch[nlohmann::json::json_pointer(json_ptr_path)] = value;
            }
        }

        return patch;
    }

    /**
     * @brief Build a ConfigStruct by merging a JSON string with command-line
     *        arguments, the latter taking priority.
     *
     * The merge is performed in two steps:
     *   1. The JSON string is applied on top of the struct defaults.
     *   2. The argv overrides are applied on top of the result (higher priority).
     *
     * @param json_str  Base configuration as a JSON string.
     * @param argc      Argument count (as received by main).
     * @param argv      Argument vector (as received by main).
     * @return          A ConfigStruct with json_str values overridden by argv.
     */
    template<typename ConfigStruct>
    ConfigStruct from_json_string_and_argv(const std::string& json_str,
                                           int argc, char* argv[]) {
        ConfigStruct default_obj;
        nlohmann::json full_j = default_obj;
        full_j.merge_patch(nlohmann::json::parse(json_str)); // base JSON
        full_j.merge_patch(argv_to_json_patch(argc, argv));  // argv wins
        return full_j.get<ConfigStruct>();
    }

    /**
     * @brief Build a ConfigStruct from command-line arguments.
     *
     * Only arguments that start with "--" are considered.
     * Two syntaxes are supported:
     *   --key=value      (single token)
     *   --key value      (two consecutive tokens)
     *
     * Keys use dot-notation to address nested fields, exactly like
     * from_string_map (e.g. --cache.cachingMethod=lru).
     *
     * @param argc  Argument count (as received by main).
     * @param argv  Argument vector (as received by main).
     * @return      A ConfigStruct with default values overridden by argv.
     */
    template<typename ConfigStruct>
    ConfigStruct from_argv(int argc, char* argv[]) {
        return from_json_string_and_argv<ConfigStruct>("{}", argc, argv);
    }
} // namespace d4
