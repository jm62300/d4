#include "Counter.hpp"

#include <signal.h>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#include "c++/semirings/MpzComplexSemiring.hpp"
#include "c++/semirings/MpzFloatSemiring.hpp"
#include "c++/semirings/MpzIntSemiring.hpp"
#include "c++/parser/ParserDimacs.hpp"
#define private public
#include "src/methods/DpllStyleMethod.hpp"
#undef private
#include "api/result/CounterResultImpl.hpp"

namespace {

template <typename T, typename O>
std::unique_ptr<d4::api::CounterResult> countModels(const d4::OptionDpllStyleMethod& options,
                                                    const d4::ProblemManager& problem, std::ostream& out) {
  auto counter = std::make_unique<d4::DpllStyleMethod<T, O>>(options, problem, out);
  T result = counter->run();

  return std::make_unique<d4::api::CounterResultImpl<T, O>>(result, std::move(counter));
}

}  // namespace

namespace d4::api {

Counter::Counter(const parser::Formula& formula,
                 const d4::OptionDpllStyleMethod& config)
    : formula_(formula) {
  setOptions(config);
}

const d4::OptionDpllStyleMethod& Counter::getOptions() const {
  return options_;
}

d4::OptionDpllStyleMethod& Counter::getOptions() {
  return options_;
}

void Counter::setOptions(const d4::OptionDpllStyleMethod& config) {
  options_ = config;
  // Force counting operation.
  options_.operationType = d4::OperationTypeManager::getOperatorType("counting");

  if (options_.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options_.optionSpecManager.needFastNotSatisfied = true;
}

const parser::Formula& Counter::getFormula() const {
  return formula_;
}

void Counter::setFormula(const parser::Formula& formula) {
  formula_ = formula;
}

std::unique_ptr<CounterResult> Counter::run(std::ostream& out) {
  // Build the problem from the parsed formula.
  std::vector<d4::BcGate> gates;
  gates.reserve(formula_.clauses.size());
  for (auto& cl : formula_.clauses) {
    std::vector<d4::Lit> d4Clause;
    for (auto& l : cl) d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
  }

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

}  // namespace d4::api
