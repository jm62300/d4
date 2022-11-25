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
  Counter<T> *m_counter;

  ProblemManager *m_problem;
  std::ostream m_out;
  std::ostream m_outCounter;
  LastBreathPreproc m_lastBreath;
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
      : m_problem(initProblem), m_out(nullptr), m_outCounter(nullptr) {
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
     Init the counter with the projected clauses.

     @param[in] vm, the option to create the SAT solver.
     @param[in] problem, the input problem (only used to get information about
     weight).
     @param[in] isFloat, specify if the weight are float or int.
     @param[in] clauses, the set of clauses.
     @param[in] nbVar, the number of variables of the formula.
   */
  void initCounter(po::variables_map &vm, ProblemManager *problem, bool isFloat,
                   std::vector<std::vector<Lit>> &clauses, unsigned nbVar) {
    int precision = vm["float-precision"].as<int>();
#if 1
    m_outCounter.copyfmt(m_out);
    m_outCounter.clear(m_out.rdstate());
    m_outCounter.basic_ios<char>::rdbuf(m_out.rdbuf());
#else
    m_outCounter.setstate(std::ios_base::badbit);
#endif
    ProblemManagerCnf *p = new ProblemManagerCnf(nbVar, problem->getWeightLit(),
                                                 problem->getWeightVar(),
                                                 problem->getSelectedVar());
    p->setClauses(clauses);

    p->display(std::cout);

    // create the counter.
    m_out << "c [CONSTRUCTOR] Create an external counter: counting\n";
    m_counter = Counter<T>::makeCounter(vm, p, "counting", isFloat, precision,
                                        m_outCounter, m_lastBreath);
  }  // initCounter

  /**
   * @brief Run the method.
   *
   * @param vm is the options.
   */
  void run(po::variables_map &vm) {
    // prepare the counter.
    m_lastBreath.panic = 0;
    m_lastBreath.countConflict.resize(m_problem->getNbVar() + 1, 0);

    initCounter(vm, m_problem, m_isFloat,
                static_cast<ProblemManagerCnf *>(m_problem)->getClauses(),
                m_problem->getNbVar());

    // count to test.
    std::vector<Var> setOfVar(m_problem->getSelectedVar());
    std::vector<Lit> assumption;
    m_out << m_counter->count(setOfVar, assumption, m_outCounter) << "\n";
  }

  void interrupt() {}
};
}  // namespace d4