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

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationMaxTMethod.hpp"
#include "src/methods/MaxT.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionMaxTMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T, class A>
void maxT(const OptionMaxTMethod &options, ProblemManager *problem) {
  MaxT<T, A> *maxT = new MaxT<T, A>(options, problem, std::cout);

  methodRun = maxT;
  maxT->run();
  methodRun = nullptr;
  delete maxT;
}  // count

class Complex {
 public:
  mpz::mpf_float real, im;

  Complex() : real(0), im(0) {}
  Complex(mpz::mpf_float r, mpz::mpf_float i) : real(r), im(i) {}
  Complex(const std::vector<std::string> &v) {
    assert(v.size() == 2);
    real = mpz::mpf_float(v[0]);
    im = mpz::mpf_float(v[1]);
  }

  Complex operator*(Complex const &obj) {
    Complex res(real * obj.real - im * obj.im, real * obj.im + im * obj.real);
    return res;
  }

  Complex operator+(Complex const &obj) {
    Complex res(real + obj.real, im + obj.im);
    return res;
  }

  inline mpz::mpf_float norm() const { return real * real + im * im; }

  bool operator==(Complex const &obj) {
    return real == obj.real && im == obj.im;
  }

  bool operator<(Complex const &obj) { return norm() < obj.norm(); }
  bool operator>(Complex const &obj) { return norm() > obj.norm(); }
  bool operator<=(Complex const &obj) { return norm() <= obj.norm(); }
  bool operator>=(Complex const &obj) { return norm() >= obj.norm(); }

  friend std::ostream &operator<<(std::ostream &os, const Complex &dt) {
    if (dt.im < 0)
      os << dt.real << dt.im << 'i';
    else
      os << dt.real << "+" << dt.im << 'i';
    return os;
  }
};

class AggregateComplex {
 private:
  std::vector<mpz::mpf_float> &m_realLits;
  std::vector<mpz::mpf_float> &m_imLits;

 public:
  AggregateComplex(ProblemManager *problem)
      : m_realLits(problem->getWeightLit()),
        m_imLits(problem->getWeightLitIm()) {}

  inline bool isGreaterThan(Lit l1, Lit l2) {
    mpz::mpf_float n1 = m_realLits[l1.intern()] * m_realLits[l1.intern()] +
                        m_imLits[l1.intern()] * m_imLits[l1.intern()];
    mpz::mpf_float n2 = m_realLits[l2.intern()] * m_realLits[l2.intern()] +
                        m_imLits[l2.intern()] * m_imLits[l2.intern()];

    return n1 > n2;
  }

  inline Complex getWeightLit(Lit l) {
    assert(m_imLits.size() > l.intern());
    assert(m_realLits.size() > l.intern());
    return Complex(m_realLits[l.intern()], m_imLits[l.intern()]);
  }

  inline Complex getWeightVar(Var v) {
    Lit l = Lit::makeLitTrue(v);
    Complex v1(m_realLits[l.intern()], m_imLits[l.intern()]);
    Complex v2(m_realLits[(~l).intern()], m_imLits[(~l).intern()]);

    return v1 + v2;
  }  // getWeightVar

  inline void multiplyUnitFree(Complex &out, std::vector<Lit> &units,
                               std::vector<Var> &free) {
    for (auto &l : units) out = out * getWeightLit(l);
    for (auto &v : free) out = out * getWeightVar(v);
  }

  inline void setCount(mpz::mpf_float &out, mpz::mpf_float val) { out = val; }
  inline Complex sumIdentity() { return Complex(0, 0); }
  inline Complex mulIdentity() { return Complex(1, 0); }
  inline Complex min() { return Complex(0, 0); }
};

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

class AggregateMpfFloat {
 private:
  std::vector<mpz::mpf_float> &m_weightLits;

 public:
  AggregateMpfFloat(ProblemManager *problem)
      : m_weightLits(problem->getWeightLit()) {}

  inline bool isGreaterThan(Lit l1, Lit l2) {
    return m_weightLits[l1.intern()] > m_weightLits[l2.intern()];
  }

  inline BigFloat getWeightLit(Lit l) {
    return BigFloat(m_weightLits[l.intern()]);
  }

  inline BigFloat getWeightVar(Var v) {
    Lit l = Lit::makeLitTrue(v);
    return BigFloat(m_weightLits[l.intern()] + m_weightLits[(~l).intern()]);
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
 * @brief couterDemo implementation.
 */
void maxT(const po::variables_map &vm, ProblemManager *problem) {
  // get the configuration.
  ConfigurationMaxTMethod config;

  config.solver.solverName = MINISAT_CNF;

  config.cacheManagerMax.isActivated = true;
  config.cacheManagerMax.sizeFirstPage = 1UL << 30;

  config.cacheManagerInd.isActivated = true;
  config.cacheManagerInd.sizeFirstPage = 1UL << 30;

  config.branchingHeuristicMax.branchingHeuristicType = BRANCHING_CLASSIC;
  config.branchingHeuristicInd.branchingHeuristicType = BRANCHING_CLASSIC;
  config.branchingHeuristicInd.configurationPartialOrderHeuristic.verbosity =
      false;

  config.branchingHeuristicMax.scoringMethodType = SCORE_VSADS;
  config.branchingHeuristicInd.scoringMethodType = SCORE_VSADS;

  config.phaseHeuristicMax = vm["phaseHeuristicMax"].as<std::string>();

  std::string s = vm["threshold"].as<std::string>();
  std::stringstream ss(s);
  std::string t;

  // Splitting the str string by delimiter
  while (getline(ss, t, ' '))
    if (t.size()) config.thresholdList.push_back(t);

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  OptionMaxTMethod options(config);

  if (vm["complex"].as<bool>()) {
    std::cout << "c Run with the complex mode\n";
    maxT<Complex, AggregateComplex>(options, problem);
  } else {
    std::cout << "c Run with the classic mode\n";
    maxT<BigFloat, AggregateMpfFloat>(options, problem);
  }
}  // counterDemo