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

#include "Configuration.hpp"
#include "MethodManager.hpp"
#include "src/methods/Counter.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "src/problem/cnf/ProblemManagerErosionCnf.hpp"

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
   * @brief Apply the erosion process on the clauses.
   *
   * @param[in,out] clauses is the set of clauses (we suppose that the clauses
   * are sorted).
   * @return true if the problem is not trivially SAT, false otherwise.
   */
  bool erode(std::vector<std::vector<Lit>> &clauses, int nbVar) {
    std::vector<std::vector<Lit>> tmpClauses = clauses;
    std::vector<unsigned long> hashValue;
    clauses.clear();

    // generate.
    for (auto &c : tmpClauses) {
      if (c.size() == 1) return false;

      // generate the clauses.
      for (unsigned i = 0; i < c.size(); i++) {
        clauses.push_back(std::vector<Lit>());
        std::vector<Lit> &cl = clauses.back();
        cl.reserve(c.size());
        unsigned long currentHash = 0;

        for (unsigned j = 0; j < c.size(); j++) {
          if (i == j) continue;
          cl.push_back(c[j]);
          currentHash |= 1 << (c[j].intern() & 63);
        }
        hashValue.push_back(currentHash);
      }
    }

    // reduce.
    std::vector<bool> marked(2 * (nbVar + 1), false);
    for (unsigned i = 0; i < clauses.size(); i++) {
      std::vector<Lit> &c = clauses[i];
      for (auto &l : c) marked[l.intern()] = true;

      // check if the clause is already subsume.
      bool isSubsume = false;
      for (unsigned j = 0; j < i; j++) {
        std::vector<Lit> &d = clauses[j];
        if (!d.size()) continue;
        isSubsume = true;
        for (auto &l : d) {
          isSubsume = marked[l.intern()];
          if (!isSubsume) break;
        }
        if (isSubsume) break;
      }

      // restore mark.
      for (auto &l : c) marked[l.intern()] = false;
      if (isSubsume) c.clear();
    }

    unsigned j = 0;
    for (unsigned i = 0; i < clauses.size(); i++)
      if (clauses[i].size()) clauses[j++] = clauses[i];
    clauses.resize(j);

    return true;
  }  // erode

  /**
   * @brief Run the method.
   *
   * @param vm is the options.
   */
  void run(po::variables_map &vm) {
    // preprare the stream.
    std::ostream outCounter(nullptr);
    outCounter.setstate(std::ios_base::badbit);

    bool isUnsat = false;
    int nbErosion = m_depth < 0 ? m_problem->getNbVar() : m_depth;

    // the CNF formula.
    std::vector<std::vector<Lit>> softClauses =
        static_cast<ProblemManagerErosionCnf *>(m_problem)->getSoftClauses();
    std::vector<std::vector<Lit>> hardClauses =
        static_cast<ProblemManagerErosionCnf *>(m_problem)->getHardClauses();

    // require to init the solver
    std::vector<Var> setOfVar;
    for (unsigned i = 1; i < m_problem->getNbVar() + 1; i++)
      setOfVar.push_back(i);

    // iterate until the number of erosion realized is less than a given value
    // or until the formula is UNSAT.
    T lastCount = T(0);
    for (int cptErosion = 0; cptErosion <= nbErosion; cptErosion++) {
      if (isUnsat) {
        m_out << "c Erosion finished because empty clause.\n";
        m_out << "s " << cptErosion << " " << 0 << " \n";
        break;
      }

      // prepare the counter.
      ProblemManagerCnf *p = new ProblemManagerCnf(
          m_problem->getNbVar(), m_problem->getWeightLit(),
          m_problem->getWeightVar(), m_problem->getSelectedVar());
      std::vector<std::vector<Lit>> tmpClauses = softClauses;

      // add the theory clauses.
      for (auto &cl : hardClauses) tmpClauses.push_back(cl);

      p->setClauses(tmpClauses);

      // create the counter.
      outCounter << "c [CONSTRUCTOR] Create an external counter: counting\n";
      Configuration config;
      config.methodName = METH_COUNTING;
      Counter<T> *counter =
          Counter<T>::makeCounter(vm, p, config, m_isFloat,
                                  vm["float-precision"].as<int>(), outCounter);

      // count to test.
      std::vector<Lit> assumption;
      T count = counter->count(setOfVar, assumption, outCounter);
      m_out << "s " << cptErosion << " " << count << "\n";
      delete counter;

      if (cptErosion) assert(count <= lastCount);
      lastCount = count;

      if (count == 0) break;

      // erosion step.
      isUnsat = !erode(softClauses, m_problem->getNbVar());
    }
  }

  void interrupt() {}
};
}  // namespace d4