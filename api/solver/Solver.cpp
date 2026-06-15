#include "Solver.hpp"

#include <signal.h>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <boost/multiprecision/gmp.hpp>
#include <cassert>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#include "c++/semirings/MpzComplexSemiring.hpp"
#include "c++/semirings/MpzFloatSemiring.hpp"
#include "c++/semirings/MpzIntSemiring.hpp"
#include "c++/semirings/DecDNNFSemiring.hpp"
#include "c++/parser/ParserDimacs.hpp"

#define private public
#include "src/methods/DpllStyleMethod.hpp"
#undef private

#include "api/result/SolverResultImpl.hpp"

namespace {

template <typename T, typename O>
std::unique_ptr<d4::api::CountResult> countModels(const d4::OptionDpllStyleMethod& options,
                                                  const d4::ProblemManager& problem, std::ostream& out) {
  auto counter = std::make_unique<d4::DpllStyleMethod<T, O>>(options, problem, out);
  T result = counter->run();

  return std::make_unique<d4::api::CountResultImpl<T, O>>(result, std::move(counter));
}

}  // namespace

namespace d4::api {

Solver::Solver(const parser::Formula& formula,
               const d4::OptionDpllStyleMethod& config)
    : formula_(formula) {
  setOptions(config);
}

const d4::OptionDpllStyleMethod& Solver::getOptions() const {
  return options_;
}

void Solver::setOptions(const d4::OptionDpllStyleMethod& config) {
  options_ = config;
  // Force counting operation.
  options_.operationType = d4::OperationTypeManager::getOperatorType("counting");

  if (options_.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options_.optionSpecManager.needFastNotSatisfied = true;
}

const parser::Formula& Solver::getFormula() const {
  return formula_;
}

void Solver::setFormula(const parser::Formula& formula) {
  formula_ = formula;
}

void Solver::setWeights(const std::map<int, std::string>& weights, parser::WeightType type) {
  formula_.weightType = type;
  formula_.weightMap.clear();
  for (const auto& [lit, w] : weights) {
    formula_.weightMap[lit] = w;
  }
}

std::vector<d4::BcGate> Solver::buildGates() const {
  std::vector<d4::BcGate> gates;
  if (formula_.type == "circuit") {
    gates.reserve(formula_.gates.size());
    for (const auto& g : formula_.gates) {
      d4::BcGateType t;
      switch (g.gateType) {
        case parser::GateType::AND:
          t = d4::BcGateType::AND;
          break;
        case parser::GateType::OR:
          t = d4::BcGateType::OR;
          break;
        case parser::GateType::IDENTITY:
          t = d4::BcGateType::IDENTITY;
          break;
        case parser::GateType::CLAUSE:
          t = d4::BcGateType::CLAUSE;
          break;
        default:
          throw std::runtime_error("Unsupported gate type for counting");
      }
      std::vector<d4::Lit> lits;
      lits.reserve(g.inputs.size());
      for (int l : g.inputs)
        lits.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
      d4::Lit out = (g.output == 0)
                        ? d4::lit_Undef
                        : d4::Lit::makeLit(std::abs(g.output), g.output < 0);
      gates.push_back({lits, out, t});
    }
  } else {
    gates.reserve(formula_.clauses.size());
    for (const auto& cl : formula_.clauses) {
      std::vector<d4::Lit> d4Clause;
      for (auto& l : cl)
        d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
      gates.push_back({d4Clause, d4::lit_Undef, d4::BcGateType::CLAUSE});
    }
  }
  return gates;
}

std::unique_ptr<CountResult> Solver::count(std::ostream& out) {
  // Build the problem from the parsed formula.
  std::vector<d4::BcGate> gates = buildGates();

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, weight] : formula_.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  d4::ProblemManager problem(formula_.type, formula_.nbVar,
                              formula_.quantifications, weightMap, gates,
                              out);

  switch (formula_.weightType) {
    case parser::WeightType::INT:
      return countModels<mpz::mpz_int, semiring::MpzIntSemiring>(
          options_, problem, out);
    case parser::WeightType::FLOAT:
      return countModels<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options_, problem, out);
    case parser::WeightType::COMPLEX:
      return countModels<semiring::Complex, semiring::MpzComplexSemiring>(
          options_, problem, out);
  }

  throw std::runtime_error("Unknown weight type in formula");
}

std::unique_ptr<CompileResult> Solver::compile(std::ostream& out) {
  // Build the problem from the parsed formula.
  std::vector<d4::BcGate> gates = buildGates();

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, weight] : formula_.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  d4::ProblemManager problem(formula_.type, formula_.nbVar,
                              formula_.quantifications, weightMap, gates,
                              out);

  // Compile using DecDNNFSemiring
  auto compilerEngine = std::make_unique<d4::DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>>(
      options_, problem, out);
  semiring::Node resultNode = compilerEngine->run();

  return std::make_unique<CompileResultImpl>(resultNode, std::move(compilerEngine), formula_.nbVar, weightMap);
}

}  // namespace d4::api
