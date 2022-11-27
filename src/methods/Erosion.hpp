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
#pragma once

#include <boost/program_options.hpp>

#include "MethodManager.hpp"
#include "src/methods/Counter.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {
namespace po = boost::program_options;

template <class T>
class Erosion : public MethodManager {
 private:
  ProblemManager *m_problem;
  std::ostream m_out;
  int m_depth;
  bool m_isFloat;

 public:
  /**
   * @brief Create an Erosion object.
   *
   * @param vm give the options.
   * @param isFloat specifies if the problem is weighted.
   * @param initProblem the problem we are considering.
   */
  Erosion(po::variables_map &vm, bool isFloat, ProblemManager *initProblem)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(std::cout);
    m_out.clear(std::cout.rdstate());
    m_out.basic_ios<char>::rdbuf(std::cout.rdbuf());

    // set the options.
    m_depth = vm["erosion-option-depth"].as<int>();
    m_isFloat = vm["float"].as<bool>();
    m_out << "c [CONSTRUCTOR] Erosion: depth(" << m_depth << ") isFloat("
          << m_isFloat << ")\n";
  }  // constructor

 private:
  /**
   * @brief Run the method.
   *
   * @param vm is the options.
   */
  void run(po::variables_map &vm) {
    // preprare the stream.
    std::ostream outCounter(nullptr);
#if DEBUG
    outCounter.copyfmt(std::cout);
    outCounter.clear(m_out.rdstate());
    outCounter.basic_ios<char>::rdbuf(m_out.rdbuf());
#else
    outCounter.setstate(std::ios_base::badbit);
#endif

    outCounter.setstate(std::ios_base::badbit);

    bool isUnsat = false;
    int nbErosion = vm["erosion-option-depth"].as<int>();
    if (nbErosion < 0) nbErosion = m_problem->getNbVar();

    // the CNF formula.
    std::vector<std::vector<Lit>> clauses =
        static_cast<ProblemManagerCnf *>(m_problem)->getClauses();

    // require to init the solver
    LastBreathPreproc lastBreath(0, m_problem->getNbVar() + 1);
    std::vector<Var> setOfVar;
    for (unsigned i = 1; i < m_problem->getNbVar() + 1; i++)
      setOfVar.push_back(i);

    // iterate until the number of erosion realized is less than a given value
    // or until the formula is UNSAT.
    for (int cptErosion = 0; cptErosion <= nbErosion; cptErosion++) {
      if (isUnsat) {
        std::cout << "s " << cptErosion << " " << 0 << " \n";
        break;
      }

      // prepare the counter.
      ProblemManagerCnf *p = new ProblemManagerCnf(
          m_problem->getNbVar(), m_problem->getWeightLit(),
          m_problem->getWeightVar(), m_problem->getSelectedVar());
      p->setClauses(clauses);

      // create the counter.
      outCounter << "c [CONSTRUCTOR] Create an external counter: counting\n";
      Counter<T> *counter = Counter<T>::makeCounter(
          vm, p, "counting", m_isFloat, vm["float-precision"].as<int>(),
          outCounter, lastBreath);

      // count to test.
      std::vector<Lit> assumption;
      T count = counter->count(setOfVar, assumption, outCounter);
      m_out << "s " << cptErosion << " " << count << "\n";
      delete counter;

      if (count == 0) break;

      // erosion step.
      std::vector<std::vector<Lit>> tmpClauses = clauses;
      clauses.clear();

      for (auto &c : tmpClauses) {
        if (c.size() == 1) {
          isUnsat = true;
          break;
        }

        for (unsigned i = 0; i < c.size(); i++) {
          clauses.push_back(std::vector<Lit>());
          std::vector<Lit> &cl = clauses.back();
          cl.reserve(c.size());

          for (unsigned j = 0; j < c.size(); j++) {
            if (i == j) continue;
            cl.push_back(c[j]);
          }
        }
      }
    }
  }

  void interrupt() {}
};
}  // namespace d4