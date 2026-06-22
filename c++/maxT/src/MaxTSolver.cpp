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

#include "MaxTSolver.hpp"

#include <signal.h>

#include <cassert>
#include <sstream>

#include "../semirings/MpzComplexSemiring.hpp"
#include "src/methods/MaxT.hpp"
#include "src/methods/MethodManager.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T, class A>
static void runMaxT(const OptionMaxTMethod &options, ProblemManager *problem) {
  MaxT<T, A> *solver = new MaxT<T, A>(options, problem, std::cout);

  methodRun = solver;
  solver->run();
  methodRun = nullptr;
  delete solver;
}  // runMaxT

class BigFloat {
 public:
  mpz::mpf_float val;

  BigFloat() : val(0) {}
  BigFloat(mpz::mpf_float v) : val(v) {}
  BigFloat(const std::vector<std::string> &v) {
    assert(v.size() == 1);
    val = mpz::mpf_float(v[0]);
  }

  BigFloat operator*(BigFloat const &obj) {
    BigFloat res(val * obj.val);
    return res;
  }

  BigFloat operator+(BigFloat const &obj) {
    BigFloat res(val + obj.val);
    return res;
  }

  inline mpz::mpf_float norm() const { return val > 0 ? val : -val; }

  bool operator==(BigFloat const &obj) { return val == obj.val; }
  bool operator<(BigFloat const &obj) { return val < obj.val; }
  bool operator>(BigFloat const &obj) { return val > obj.val; }
  bool operator<=(BigFloat const &obj) { return val <= obj.val; }
  bool operator>=(BigFloat const &obj) { return val >= obj.val; }
  friend std::ostream &operator<<(std::ostream &os, const BigFloat &dt) {
    os << dt.val;
    return os;
  }
};

/**
 * @brief Aggregator for complex-weighted formulas (e.g. quantum amplitudes).
 * Builds its own per-literal weight array from the problem's weight map,
 * the same way semiring::MpzComplexSemiring does.
 */
class AggregateComplex {
 private:
  std::vector<semiring::Complex> m_weightLit;

 public:
  AggregateComplex(ProblemManager *problem) {
    unsigned nbVar = problem->getNbVar();
    m_weightLit.resize(2 * (nbVar + 1), semiring::Complex(1, 0));
    for (const auto &[lit, weight] : problem->getWeightMap())
      m_weightLit[lit.intern()] = semiring::Complex(weight);
  }

  inline bool isGreaterThan(Lit l1, Lit l2) {
    return m_weightLit[l1.intern()].norm() > m_weightLit[l2.intern()].norm();
  }

  inline semiring::Complex getWeightLit(Lit l) {
    return m_weightLit[l.intern()];
  }

  inline semiring::Complex getWeightVar(Var v) {
    Lit l = Lit::makeLitTrue(v);
    return m_weightLit[l.intern()] + m_weightLit[(~l).intern()];
  }  // getWeightVar

  inline void multiplyUnitFree(semiring::Complex &out, std::vector<Lit> &units,
                               std::vector<Var> &free) {
    for (auto &l : units) out = out * getWeightLit(l);
    for (auto &v : free) out = out * getWeightVar(v);
  }

  inline semiring::Complex sumIdentity() { return semiring::Complex(0, 0); }
  inline semiring::Complex mulIdentity() { return semiring::Complex(1, 0); }
  inline semiring::Complex min() { return semiring::Complex(0, 0); }
};

/**
 * @brief Aggregator for real-weighted (or unweighted) formulas. Builds its
 * own per-literal weight array from the problem's weight map, the same way
 * semiring::MpzFloatSemiring does.
 */
class AggregateMpfFloat {
 private:
  std::vector<mpz::mpf_float> m_weightLit;

 public:
  AggregateMpfFloat(ProblemManager *problem) {
    unsigned nbVar = problem->getNbVar();
    m_weightLit.resize(2 * (nbVar + 1), mpz::mpf_float(1));
    for (const auto &[lit, weight] : problem->getWeightMap())
      m_weightLit[lit.intern()] = mpz::mpf_float(weight);
  }

  inline bool isGreaterThan(Lit l1, Lit l2) {
    return m_weightLit[l1.intern()] > m_weightLit[l2.intern()];
  }

  inline BigFloat getWeightLit(Lit l) {
    return BigFloat(m_weightLit[l.intern()]);
  }

  inline BigFloat getWeightVar(Var v) {
    Lit l = Lit::makeLitTrue(v);
    return BigFloat(m_weightLit[l.intern()] + m_weightLit[(~l).intern()]);
  }

  inline void multiplyUnitFree(BigFloat &out, std::vector<Lit> &units,
                               std::vector<Var> &free) {
    for (auto &l : units) out = out * getWeightLit(l);
    for (auto &v : free) out = out * getWeightVar(v);
  }

  inline BigFloat sumIdentity() { return BigFloat(mpz::mpf_float(0)); }
  inline BigFloat mulIdentity() { return BigFloat(mpz::mpf_float(1)); }
  inline BigFloat min() { return BigFloat(mpz::mpf_float(0)); }
};

/**
 * @brief maxT implementation.
 */
void maxT(const d4::OptionMaxTMethod &inputOptions,
         const parser::Formula &formula) {
  OptionMaxTMethod options = inputOptions;

  options.optionSolver.solverName = MINISAT_CNF;

  options.optionCacheManagerMax.isActivated = true;
  options.optionCacheManagerMax.optionBucketManager.sizeFirstPage = 1UL << 30;

  options.optionCacheManagerInd.isActivated = true;
  options.optionCacheManagerInd.optionBucketManager.sizeFirstPage = 1UL << 30;

  options.optionBranchingHeuristicMax.branchingHeuristicType =
      BRANCHING_CLASSIC;
  options.optionBranchingHeuristicInd.branchingHeuristicType =
      BRANCHING_CLASSIC;

  options.optionBranchingHeuristicMax.scoringMethodType = SCORE_VSADS;
  options.optionBranchingHeuristicInd.scoringMethodType = SCORE_VSADS;

  std::stringstream ss(options.threshold.get());
  std::string t;
  while (std::getline(ss, t, ' '))
    if (t.size()) options.thresholdList.push_back(t);

  // build the problem; heap-allocated since MaxT takes ownership (its
  // destructor deletes the pointer it is given).
  std::vector<BcGate> gates;
  gates.reserve(formula.clauses.size());
  for (auto &cl : formula.clauses) {
    std::vector<Lit> d4Clause;
    for (auto &l : cl) d4Clause.push_back(Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({d4Clause, lit_Undef, BcGateType::CLAUSE});
  }

  std::map<Lit, std::string> weightMap;
  for (const auto &[lit, weight] : formula.weightMap)
    weightMap[Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  ProblemManager *problem =
      new ProblemManager(formula.type, formula.nbVar, formula.quantifications,
                         weightMap, gates, std::cout);

  MethodManager::displayInfoVariables(*problem, std::cout);

  if (formula.weightType == parser::WeightType::COMPLEX) {
    std::cout << "c Run with the complex mode\n";
    runMaxT<semiring::Complex, AggregateComplex>(options, problem);
  } else {
    std::cout << "c Run with the classic mode\n";
    runMaxT<BigFloat, AggregateMpfFloat>(options, problem);
  }
}  // maxT
