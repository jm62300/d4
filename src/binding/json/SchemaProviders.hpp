#pragma once

#include <nlohmann/json.hpp>
#include "src/binding/json/Binding.hpp"
#include "src/binding/json/SchemaGenerator.hpp"

namespace d4 {

    template<>
    struct SchemaProvider<ConfigurationPeproc> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationPeproc"},
                {"properties", {
                    {"preprocMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"inputType", {
                        {"description", "The input type"},
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"nbIteration", {
                        {"description", "The number of iterations for the preprocessor."},
                        {"type", "integer"}
                    }},
                    {"ordered", {
                        {"description", "Set to true if the elimination need to follow some order."},
                        {"type", "boolean"}
                    }},
                    {"onlyUseGates", {
                        {"type", "boolean"}
                    }},
                    {"strongElim", {
                        {"description", "If set to true, then the variable are elminated whatever the impact on the size of the formula."},
                        {"type", "boolean"}
                    }},
                    {"timeout", {
                        {"description", "The time in second given to the preproc (0 means no timeout)."},
                        {"type", "integer"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<Configuration> {
        static nlohmann::json get() {
            return {
                {"title", "Configuration"},
                {"properties", {
                    {"methodName", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"precision", {
                        {"description", "The precision for the float."},
                        {"type", "integer"}
                    }},
                    {"isFloat", {
                        {"type", "boolean"}
                    }},
                    {"inputName", {
                        {"type", "string"}
                    }},
                    {"problemInputType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"configurationPreproc", {
                        {"type", "object"},
                        {"title", "ConfigurationPeproc"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationQbfCounter> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationQbfCounter"},
                {"properties", {
                    {"cache", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"spec", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"branchingHeuristic", {
                        {"description", "The branching heuristic used (classic or large-clause if d4 selects first literals in large clauses.)"},
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"partitioningHeuristic", {
                        {"type", "object"},
                        {"title", "ConfigurationPartialOrderHeuristic"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationMaxSharpSatMathod> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationMaxSharpSatMathod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"digOnAnd", {
                        {"type", "boolean"}
                    }},
                    {"threshold", {
                        {"description", "Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i)."},
                        {"type", "number"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"specManager", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"phaseHeuristicMax", {
                        {"type", "string"}
                    }},
                    {"randomPhaseHeuristicMax", {
                        {"type", "integer"}
                    }},
                    {"branchingHeuristicMax", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerMax", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"branchingHeuristicInd", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerInd", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationMinSharpSatMathod> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationMinSharpSatMathod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"digOnAnd", {
                        {"type", "boolean"}
                    }},
                    {"threshold", {
                        {"description", "Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i)."},
                        {"type", "number"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"specManager", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"phaseHeuristicMin", {
                        {"type", "string"}
                    }},
                    {"randomPhaseHeuristicMin", {
                        {"type", "integer"}
                    }},
                    {"branchingHeuristicMin", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerMin", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"branchingHeuristicInd", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerInd", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationPartialOrderHeuristic> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationPartialOrderHeuristic"},
                {"properties", {
                    {"partialOrderMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"partitionerName", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"treeDecompositionMethod", {
                        {"description", "The tool used for computing the tree decomposition (flow-cutter)."},
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"treeDecompositionerMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"hyperGraphExtractorMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"graphExtractorMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"useSimpGraphExtractor", {
                        {"type", "boolean"}
                    }},
                    {"budget", {
                        {"description", "The maximum allowed computation time (in seconds) for tree decomposition."},
                        {"type", "integer"}
                    }},
                    {"seed", {
                        {"description", "A random seed for methods incorporating randomness."},
                        {"type", "integer"}
                    }},
                    {"verbosity", {
                        {"description", "Control if the tool for computing the tree-decompositio will be verbose or not."},
                        {"type", "boolean"}
                    }},
                    {"givenOrder", {
                        {"type", "array"}
                    }},
                    {"scaleFactor", {
                        {"type", "number"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationBranchingHeuristic> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationBranchingHeuristic"},
                {"properties", {
                    {"configurationPartialOrderHeuristic", {
                        {"type", "object"},
                        {"title", "ConfigurationPartialOrderHeuristic"}
                    }},
                    {"scoringMethodType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"phaseHeuristicType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"branchingHeuristicType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"reversePhase", {
                        {"type", "boolean"}
                    }},
                    {"freqDecay", {
                        {"description", "Gives the decay frequency"},
                        {"type", "integer"}
                    }},
                    {"limitSizeClause", {
                        {"type", "integer"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationEREMethod> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationEREMethod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"digOnAnd", {
                        {"type", "boolean"}
                    }},
                    {"threshold", {
                        {"description", "Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i)."},
                        {"type", "number"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"specManager", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"cutExist", {
                        {"type", "boolean"}
                    }},
                    {"randomPhaseHeuristicExist", {
                        {"type", "integer"}
                    }},
                    {"phaseHeuristicBestExist", {
                        {"type", "boolean"}
                    }},
                    {"branchingHeuristicExist", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerExist", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"branchingHeuristicRandom", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerRandom", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"computeComponentOnRandom", {
                        {"type", "boolean"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationProjMcMethod> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationProjMcMethod"},
                {"properties", {
                    {"refinement", {
                        {"type", "boolean"}
                    }},
                    {"cache", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"specs", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"counter", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationMaxTMethod> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationMaxTMethod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"specManager", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"phaseHeuristicMax", {
                        {"type", "string"}
                    }},
                    {"randomPhaseHeuristicMax", {
                        {"type", "integer"}
                    }},
                    {"branchingHeuristicMax", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerMax", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"branchingHeuristicInd", {
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"cacheManagerInd", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"thresholdList", {
                        {"type", "array"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationDpllStyleMethod> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationDpllStyleMethod"},
                {"properties", {
                    {"operationType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"cache", {
                        {"type", "object"},
                        {"title", "ConfigurationCache"}
                    }},
                    {"solver", {
                        {"description", "The solver we will use"},
                        {"type", "object"},
                        {"title", "ConfigurationSolver"}
                    }},
                    {"spec", {
                        {"type", "object"},
                        {"title", "ConfigurationSpec"}
                    }},
                    {"branchingHeuristic", {
                        {"description", "The branching heuristic used (classic or large-clause if d4 selects first literals in large clauses.)"},
                        {"type", "object"},
                        {"title", "ConfigurationBranchingHeuristic"}
                    }},
                    {"exploitModel", {
                        {"type", "boolean"}
                    }},
                    {"verbosity", {
                        {"description", "Control if the tool for computing the tree-decompositio will be verbose or not."},
                        {"type", "boolean"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationSolver> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationSolver"},
                {"properties", {
                    {"solverName", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationCache> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationCache"},
                {"properties", {
                    {"cachingMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"cacheCleaningStrategy", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"modeStore", {
                        {"type", "object"},
                        {"title", "ModeStore"}
                    }},
                    {"clauseRepresentation", {
                        {"description", "The way the clause are represented in the cache (combi, sym, clause and index)."},
                        {"type", "object"},
                        {"title", "ClauseRepresentation"}
                    }},
                    {"isActivated", {
                        {"type", "boolean"}
                    }},
                    {"sizeFirstPage", {
                        {"description", "The block size of memory allocated for the first page of the cache structure."},
                        {"type", "object"},
                        {"title", "unsigned long"}
                    }},
                    {"sizeAdditionalPage", {
                        {"description", "The block size of memory allocated for the next page of the cache structure."},
                        {"type", "object"},
                        {"title", "unsigned long"}
                    }},
                    {"limitVarSym", {
                        {"type", "integer"}
                    }},
                    {"limitVarIndex", {
                        {"type", "integer"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<ConfigurationSpec> {
        static nlohmann::json get() {
            return {
                {"title", "ConfigurationSpec"},
                {"properties", {
                    {"specUpdateType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"removeGates", {
                        {"description", "If this option is activated and if the problem is a circuit, then some gates can be removed during the search if those ones are not active."},
                        {"type", "boolean"}
                    }},
                    {"needFastNotSatisfied", {
                        {"type", "boolean"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionSolver> {
        static nlohmann::json get() {
            return {
                {"title", "OptionSolver"},
                {"properties", {
                    {"solverName", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionBucketManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionBucketManager"},
                {"properties", {
                    {"modeStore", {
                        {"type", "object"},
                        {"title", "ModeStore"}
                    }},
                    {"clauseRepresentation", {
                        {"description", "The way the clause are represented in the cache (combi, sym, clause and index)."},
                        {"type", "object"},
                        {"title", "ClauseRepresentation"}
                    }},
                    {"sizeFirstPage", {
                        {"description", "The block size of memory allocated for the first page of the cache structure."},
                        {"type", "object"},
                        {"title", "unsigned long"}
                    }},
                    {"sizeAdditionalPage", {
                        {"description", "The block size of memory allocated for the next page of the cache structure."},
                        {"type", "object"},
                        {"title", "unsigned long"}
                    }},
                    {"limitNbVarSym", {
                        {"type", "integer"}
                    }},
                    {"limitNbVarIndex", {
                        {"type", "integer"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionCacheManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionCacheManager"},
                {"properties", {
                    {"cachingMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionBucketManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionCacheCleaningManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"isActivated", {
                        {"type", "boolean"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionCacheCleaningManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionCacheCleaningManager"},
                {"properties", {
                    {"cacheCleaningStrategy", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionBranchingHeuristic> {
        static nlohmann::json get() {
            return {
                {"title", "OptionBranchingHeuristic"},
                {"properties", {
                    {"optionPartialOrderHeuristic", {
                        {"type", "object"},
                        {"title", "OptionPartialOrderHeuristic"}
                    }},
                    {"scoringMethodType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"phaseHeuristicType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"branchingHeuristicType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"reversePhase", {
                        {"type", "boolean"}
                    }},
                    {"freqDecay", {
                        {"description", "Gives the decay frequency"},
                        {"type", "integer"}
                    }},
                    {"limitSizeClause", {
                        {"type", "integer"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionSpecManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionSpecManager"},
                {"properties", {
                    {"specUpdateType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"removeGates", {
                        {"description", "If this option is activated and if the problem is a circuit, then some gates can be removed during the search if those ones are not active."},
                        {"type", "boolean"}
                    }},
                    {"needFastNotSatisfied", {
                        {"type", "boolean"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionPreprocManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionPreprocManager"},
                {"properties", {
                    {"inputType", {
                        {"description", "The input type"},
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"preprocMethod", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"nbIteration", {
                        {"type", "integer"}
                    }},
                    {"onlyUseGates", {
                        {"type", "boolean"}
                    }},
                    {"ordered", {
                        {"description", "Set to true if the elimination need to follow some order."},
                        {"type", "boolean"}
                    }},
                    {"strongElim", {
                        {"description", "If set to true, then the variable are elminated whatever the impact on the size of the formula."},
                        {"type", "boolean"}
                    }},
                    {"timeout", {
                        {"description", "The time in second given to the preproc (0 means no timeout)."},
                        {"type", "integer"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionOperationManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionOperationManager"},
                {"properties", {
                    {"operatorType", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionMaxSharpSatMethod> {
        static nlohmann::json get() {
            return {
                {"title", "OptionMaxSharpSatMethod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"digOnAnd", {
                        {"type", "boolean"}
                    }},
                    {"threshold", {
                        {"description", "Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i)."},
                        {"type", "number"}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"phaseHeuristicMax", {
                        {"type", "string"}
                    }},
                    {"randomPhaseHeuristicMax", {
                        {"type", "integer"}
                    }},
                    {"optionBranchingHeuristicMax", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerMax", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionBranchingHeuristicInd", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerInd", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionMethodManager> {
        static nlohmann::json get() {
            return {
                {"title", "OptionMethodManager"},
                {"properties", {
                    {"optionOperationManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionMaxTMethod> {
        static nlohmann::json get() {
            return {
                {"title", "OptionMaxTMethod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"thresholdList", {
                        {"type", "array"}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"phaseHeuristicMax", {
                        {"type", "string"}
                    }},
                    {"randomPhaseHeuristicMax", {
                        {"type", "integer"}
                    }},
                    {"optionBranchingHeuristicMax", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerMax", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionBranchingHeuristicInd", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerInd", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionQbfCounter> {
        static nlohmann::json get() {
            return {
                {"title", "OptionQbfCounter"},
                {"properties", {
                    {"optionCacheManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionBranchingHeuristic", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionProjMcMethod> {
        static nlohmann::json get() {
            return {
                {"title", "OptionProjMcMethod"},
                {"properties", {
                    {"refinement", {
                        {"type", "boolean"}
                    }},
                    {"optionCache", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecs", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionCounter", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionDpllStyleMethod> {
        static nlohmann::json get() {
            return {
                {"title", "OptionDpllStyleMethod"},
                {"properties", {
                    {"optionOperationManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionCacheManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionBranchingHeuristic", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"exploitModel", {
                        {"type", "boolean"}
                    }},
                    {"verbosity", {
                        {"description", "Control if the tool for computing the tree-decompositio will be verbose or not."},
                        {"type", "boolean"}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionMinSharpSatMethod> {
        static nlohmann::json get() {
            return {
                {"title", "OptionMinSharpSatMethod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"digOnAnd", {
                        {"type", "boolean"}
                    }},
                    {"threshold", {
                        {"description", "Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i)."},
                        {"type", "number"}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"phaseHeuristicMin", {
                        {"type", "string"}
                    }},
                    {"randomPhaseHeuristicMin", {
                        {"type", "integer"}
                    }},
                    {"optionBranchingHeuristicMin", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerMin", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"optionBranchingHeuristicInd", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerInd", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

    template<>
    struct SchemaProvider<OptionEREMethod> {
        static nlohmann::json get() {
            return {
                {"title", "OptionEREMethod"},
                {"properties", {
                    {"greedyInitActivated", {
                        {"type", "boolean"}
                    }},
                    {"digOnAnd", {
                        {"type", "boolean"}
                    }},
                    {"threshold", {
                        {"description", "Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i)."},
                        {"type", "number"}
                    }},
                    {"optionSolver", {
                        {"type", "object"},
                        {"title", "OptionSolver"}
                    }},
                    {"optionSpecManager", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"cutExist", {
                        {"type", "boolean"}
                    }},
                    {"phaseHeuristicBestExist", {
                        {"type", "boolean"}
                    }},
                    {"randomPhaseHeuristicExist", {
                        {"type", "integer"}
                    }},
                    {"optionBranchingHeuristicExist", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerExist", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }},
                    {"computeComponentOnRandom", {
                        {"type", "boolean"}
                    }},
                    {"optionBranchingHeuristicRandom", {
                        {"type", "object"},
                        {"title", "OptionBranchingHeuristic"}
                    }},
                    {"optionCacheManagerRandom", {
                        {"type", {
                            "integer",
                            "string"
                        }}
                    }}
                }}
            };
        }
    };

} // namespace d4
