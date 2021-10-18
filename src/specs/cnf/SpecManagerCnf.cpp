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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm> // std::sort
#include <iostream>

#include "SpecManagerCnf.hpp"
#include "SpecManagerCnfDyn.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
/**
   Constructor.
*/
SpecManagerCnf::SpecManagerCnf(int nbClause, int _nbVar, int _m_maxSizeClause)
    : m_nbVar(_nbVar), m_maxSizeClause(_m_maxSizeClause) {
  // variables:
  m_inCurrentComponent.resize(m_nbVar + 1, false);
  m_currentValue.resize(m_nbVar + 1, l_Undef);
  m_idxComponent.resize(m_nbVar + 1, 0);

  // occurrences:
  m_occListBin.resize((m_nbVar + 1) << 1, std::vector<int>());
  m_occListNotBin.resize((m_nbVar + 1) << 1, std::vector<int>());

  // clauses:
  m_mustUnMark.reserve(nbClause);
  m_markView.resize(nbClause, false);
  m_infoClauses.resize(nbClause);
} // construtor

/**
   Constructor.
*/
SpecManagerCnf::SpecManagerCnf(ProblemManager &p) : m_nbVar(p.getNbVar()) {
  initFormula(p);

  // variables:
  m_inCurrentComponent.resize(m_nbVar + 1, false);
  m_currentValue.resize(m_nbVar + 1, l_Undef);
  m_idxComponent.resize(m_nbVar + 1, 0);

  // occurrences:
  m_occListBin.resize((m_nbVar + 1) << 1, std::vector<int>());
  m_occListNotBin.resize((m_nbVar + 1) << 1, std::vector<int>());

  // clauses:
  unsigned nbClause = m_clauses.size();
  m_mustUnMark.reserve(nbClause);
  m_markView.resize(nbClause, false);

  m_infoClauses.resize(nbClause);
  m_maxSizeClause = 0;
  if (!m_clauses.size())
    return;

  // get the size of the largest clause.
  m_maxSizeClause = m_clauses[0].size();
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    std::vector<Lit> &cl = m_clauses[i];
    std::vector<std::vector<int>> &listOcc =
        (cl.size() == 2) ? m_occListBin : m_occListNotBin;
    for (auto &l : cl)
      listOcc[l.intern()].push_back(i);
    if (cl.size() > m_maxSizeClause)
      m_maxSizeClause = cl.size();
    m_infoClauses[i].watcher = cl[0];
  }

  m_infoCluster.resize(p.getNbVar() + nbClause + 1, {0, 0, -1});
} // construtor

/**
   Collect the set of literals connected to l and store the result in
   varComponent.

   @param[in] l, the considered literal
   @param[in] v, the label of the previously assigned component (0 if not
   assigned).
   @param[in] varComponent, the set of varaible connected to l.
   @param[in] nbComponent, the component label.
*/
void SpecManagerCnf::connectedToLit(Lit l, std::vector<int> &v,
                                    std::vector<Var> &varComponent,
                                    int nbComponent) {

  for (unsigned i = 0; i < 2; i++) {
    std::vector<int> &listIndex =
        (i) ? getVecIdxClauseBin(l) : getVecIdxClauseNotBin(l);

    for (auto &idx : listIndex) {
      if (m_markView[idx])
        continue;
      m_markView[idx] = true;
      m_mustUnMark.push_back(idx);

      // compute component
      for (auto &l : m_clauses[idx]) {
        if (m_currentValue[l.var()] != l_Undef || v[l.var()])
          continue;

        varComponent.push_back(l.var());
        v[l.var()] = nbComponent;
      }
    }
  }
} // connectedToLit

#if 0

/**
   Look all the formula in order to compute the connected component
   of the formula.

   @param[out] varCo, the different connected components found
   @param[in] setOfVar, the current set of variables
   @param[out] freeVar, the set of variables that are present in setOfVar but
   not in the problem anymore

   \return the number of component found
*/
int SpecManagerCnf::computeConnectedComponent(
    std::vector<std::vector<Var>> &varCo, std::vector<Var> &setOfVar,
    std::vector<Var> &freeVar) {
  freeVar.resize(0);

  varCo.push_back(std::vector<Var>());

  int nbComponent = 0;
  for (const auto v : setOfVar) {
    if (m_currentValue[v] != l_Undef || m_idxComponent[v])
      continue;

    // index a new composant
    nbComponent++;
    m_idxComponent[v] = nbComponent;

    // save the variables of connected component
    assert(!m_tmpVecVar.size());
    m_tmpVecVar.push_back(v);

    int cpt = 0;
    while (m_tmpVecVar.size()) {
      cpt++;
      Lit l = Lit::makeLit(m_tmpVecVar.back(), false);
      m_tmpVecVar.pop_back();

      if (getNbOccurrence(l))
        connectedToLit(l, m_idxComponent, m_tmpVecVar, nbComponent);
      if (getNbOccurrence(~l))
        connectedToLit(~l, m_idxComponent, m_tmpVecVar, nbComponent);
    }

    assert(cpt > 0);
    if (cpt == 1) {
      m_idxComponent[v] = 0;
      nbComponent--; // it is alone ...
    }
  }

  resetUnMark();

  varCo.resize(nbComponent);
  for (const auto v : setOfVar) {
    if (m_idxComponent[v]) {
      assert(m_idxComponent[v] <= (int)varCo.size());
      varCo[m_idxComponent[v] - 1].push_back(v);
      assert(nbComponent);
    } else if (m_currentValue[v] == l_Undef) {
      freeVar.push_back(v);
      assert(getNbClause(v) == 0);
    }

    m_idxComponent[v] = 0;
  }

  return nbComponent;
} // computeConnectedComponent

#else

#define TEST 0

/**
   Look all the formula in order to compute the connected component
   of the formula (union find algorithm).

   @param[out] varCo, the different connected components found
   @param[in] setOfVar, the current set of variables
   @param[out] freeVar, the set of variables that are present in setOfVar but
   not in the problem anymore

   \return the number of component found
*/
int SpecManagerCnf::computeConnectedComponent(
    std::vector<std::vector<Var>> &varCo, std::vector<Var> &setOfVar,
    std::vector<Var> &freeVar) {
  freeVar.resize(0);

#if TEST
  for (auto s : m_markView)
    assert(!s);
#endif

  for (auto v : setOfVar) {
    m_infoCluster[v].parent = v;
    m_infoCluster[v].size = 1;
  }

#if TEST
  std::cout << "START "
               "---------------------------------------------------------------"
               "----\n";
  for (auto &w : setOfVar) {
    Lit l = Lit::makeLit(w, false);
    for (unsigned i = 0; i < 2; i++) {
      std::cout << l << ": ";

      for (unsigned j = 0; j < 2; j++) { // both occurrence lists.
        std::vector<int> &listIndex =
            (j) ? getVecIdxClauseBin(l) : getVecIdxClauseNotBin(l);

        for (auto idx : listIndex)
          std::cout << idx << " ";
      }
      std::cout << "\n";
      l = ~l;
    }
  }
#endif

  unsigned nbComponent = setOfVar.size();
  for (auto v : setOfVar) {
    if (m_currentValue[v] != l_Undef) {
#if TEST
      std::cout << "already assigned " << v << "\n";
#endif
      continue;
    }
#if TEST
    std::cout << "considere " << v << "\n";
#endif

    // visit the index clauses
    unsigned rootV = v;
    Lit l = Lit::makeLit(v, false);

    for (unsigned i = 0; i < 2; i++) {   // both literals.
      for (unsigned j = 0; j < 2; j++) { // both occurrence lists.
        std::vector<int> &listIndex =
            (j) ? getVecIdxClauseBin(l) : getVecIdxClauseNotBin(l);

        for (auto &idx : listIndex) {
          if (!m_markView[idx]) {
            m_infoCluster[idx + m_nbVar + 1].parent = rootV;
            m_infoCluster[rootV].size++;
            m_markView[idx] = true;
            m_mustUnMark.push_back(idx);
          } else {
            // search for the root.
            unsigned rootW = m_infoCluster[idx + m_nbVar + 1].parent;
            while (rootW != m_infoCluster[rootW].parent) {
              m_infoCluster[rootW].parent =
                  m_infoCluster[m_infoCluster[rootW].parent].parent;
              rootW = m_infoCluster[rootW].parent;
            }

            // already in the same component.
            if (rootV == rootW)
              continue;
#if TEST
            std::cout << "merge " << rootV << " " << rootW << " " << idx
                      << "\n";
#endif
#if TEST
            for (unsigned i = 0; i < m_infoCluster.size(); i++) {
              std::cout << i << ": " << m_infoCluster[i].parent << " "
                        << m_infoCluster[i].size << " |";
              if (i && !(i % 10))
                std::cout << "\n";
            }
            std::cout << "\n";
#endif

            // because we merge two components.
            nbComponent--;

            // union.
            if (m_infoCluster[rootV].size < m_infoCluster[rootW].size) {
              m_infoCluster[rootV].parent = m_infoCluster[rootW].parent;
              m_infoCluster[rootW].size += m_infoCluster[rootV].size;
              rootV = rootW;
            } else {
              m_infoCluster[rootW].parent = m_infoCluster[rootV].parent;
              m_infoCluster[rootV].size += m_infoCluster[rootW].size;
            }
#if TEST
            for (unsigned i = 0; i < m_infoCluster.size(); i++) {
              std::cout << i << ": " << m_infoCluster[i].parent << " "
                        << m_infoCluster[i].size << " |";
              if (i && !(i % 10))
                std::cout << "\n";
            }
            std::cout << "\n";
#endif
          }
        }
      }
      l = ~l;
    }
  }

  // collect the component.
  std::vector<Var> rootSet;
  for (auto &v : setOfVar) {
    if (m_currentValue[v] != l_Undef)
      continue;

    if (m_infoCluster[v].parent == v && m_infoCluster[v].size == 1) {
      freeVar.push_back(v);
#if TEST
      if (getNbClause(v) != 0) {
        std::cout << "BUG : " << v << " " << setOfVar.size() << "\n";
        for (auto &w : setOfVar) {
          Lit l = Lit::makeLit(w, false);
          for (unsigned i = 0; i < 2; i++) {
            std::cout << l << "[" << (m_currentValue[v] == l_Undef) << "]: ";

            for (unsigned j = 0; j < 2; j++) { // both occurrence lists.
              std::vector<int> &listIndex =
                  (j) ? getVecIdxClauseBin(l) : getVecIdxClauseNotBin(l);

              for (auto idx : listIndex)
                std::cout << idx << " ";
            }
            std::cout << "\n";
            l = ~l;
          }
        }
      }
#endif
      assert(getNbClause(v) == 0);
      continue;
    }
    assert(getNbClause(v) != 0);
    assert(m_currentValue[v] == l_Undef);

    // get the root.
    unsigned rootV = m_infoCluster[v].parent;
    while (rootV != m_infoCluster[rootV].parent) {
      m_infoCluster[rootV].parent =
          m_infoCluster[m_infoCluster[rootV].parent].parent;
      rootV = m_infoCluster[rootV].parent;
    }

#if TEST
    std::cout << "consider: " << v << "[" << rootV << "]\n";
#endif

    if (m_infoCluster[rootV].pos == -1) {
      m_infoCluster[rootV].pos = varCo.size();
      varCo.push_back(std::vector<Var>());
      rootSet.push_back(rootV);
    }

    varCo[m_infoCluster[rootV].pos].push_back(v);
  }

  // restore for the next run.
  resetUnMark();
  for (auto &v : rootSet)
    m_infoCluster[v].pos = -1;

  return varCo.size();
} // computeConnectedComponent
#endif

/**
   Test if a given clause is actually satisfied under the current
   interpretation.

   @param[in] idx, the clause index.

   \return true if the clause is satisfied, false otherwise.
*/
bool SpecManagerCnf::isSatisfiedClause(unsigned idx) {
  assert(idx < m_clauses.size());
  return m_infoClauses[idx].nbSat;
} // isSatisfiedClause

/**
   Test if a given clause is actually satisfied under the current
   interpretation.

   @param[in] idx, the clause index.

   \return true if the clause is satisfied, false otherwise.
*/
bool SpecManagerCnf::isSatisfiedClause(std::vector<Lit> &c) {
  for (auto &l : c) {
    if (!litIsAssigned(l))
      continue;
    if (l.sign() && m_currentValue[l.var()] == l_False)
      return true;
    if (!l.sign() && m_currentValue[l.var()] == l_True)
      return true;
  }

  return false;
} // isSatisfiedClause

/**
   Test at the same time if a given clause is actually satisfied under
   the current interpretation and if its set of variables belong to the
   current component that is represented as a boolean map given in
   parameter.

   @param[in] idx, the clause index.
   @param[in] currentComponent, currentComponent[var] is true when var is in the
   current component, false otherwise.

   \return true if the clause is satisfied, false otherwise.
*/
bool SpecManagerCnf::isNotSatisfiedClauseAndInComponent(
    int idx, std::vector<bool> &m_inCurrentComponent) {
  if (m_infoClauses[idx].nbSat)
    return false;
  assert(m_infoClauses[idx].watcher != lit_Undef);
  assert(!litIsAssigned(m_infoClauses[idx].watcher));
  return m_inCurrentComponent[m_infoClauses[idx].watcher.var()];
} // isSatisfiedClause

void SpecManagerCnf::getCurrentClauses(std::vector<unsigned> &idxClauses,
                                       std::vector<Var> &component) {
  idxClauses.resize(0);
  for (auto &v : component)
    m_inCurrentComponent[v] = true;
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for (auto &v : component)
    m_inCurrentComponent[v] = false;
} // getCurrentclauses

void SpecManagerCnf::getCurrentClausesNotBin(std::vector<unsigned> &idxClauses,
                                             std::vector<Var> &component) {
  idxClauses.resize(0);
  for (auto &v : component)
    m_inCurrentComponent[v] = true;
  for (auto &i : m_clausesNotBin) {
    if (isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for (auto &v : component)
    m_inCurrentComponent[v] = false;
} // getCurrentclauses

void SpecManagerCnf::initFormula(ProblemManager &p) {
  try {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(p);
    m_clauses = pcnf.getClauses();

    for (unsigned i = 0; i < m_clauses.size(); i++)
      if (m_clauses[i].size() > 2)
        m_clausesNotBin.push_back(i);
  } catch (std::bad_cast &bc) {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }

  for (auto &val : m_currentValue)
    val = l_Undef;
} // initFormula

void SpecManagerCnf::showFormula(std::ostream &out) {
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for (auto &cl : m_clauses) {
    showListLit(out, cl);
    out << "0\n";
  }
} // showFormula

void SpecManagerCnf::showTrail(std::ostream &out) {
  for (int i = 0; i < getNbVariable(); i++) {
    if (!varIsAssigned(i))
      continue;
    Lit l = Lit::makeLit(i, false);
    if (litIsAssignedToTrue(l))
      out << l << " ";
    else
      out << ~l << " ";
  }
  out << "\n";
} // showFormula

void SpecManagerCnf::showCurrentFormula(std::ostream &out) {
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (m_infoClauses[i].nbSat)
      continue;
    for (auto &l : m_clauses[i])
      if (!litIsAssigned(l))
        out << l << " ";
    out << "0\n";
  }
} // showFormula
} // namespace d4
