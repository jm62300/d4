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

#include "src/heuristics/phaseSelection/PhaseHeuristic.hpp"
#include "src/heuristics/scoringVariable/ScoringMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"

namespace d4 {
const unsigned SIZE_PAGE_LIST_LIT = 1 << 10;

/**
 * @brief Memory allocator for managing lists of literals.
 *
 * This class handles memory allocation for storing lists of literals (Lit).
 * It optimizes memory usage by reusing previously allocated memory blocks
 * whenever possible.
 */
class ListLitAllocator {
 private:
  unsigned m_posPage = 0;  ///< Current position within the active memory page.
  Lit *m_currentPage = NULL;   ///< Pointer to the active memory page.
  std::vector<Lit *> m_pages;  ///< Stores all allocated pages for cleanup.
  std::vector<std::vector<Lit *>> m_availaible;  ///< Reusable memory blocks.

 public:
  /**
   * @brief Constructor initializes the allocator state.
   */
  ListLitAllocator() {
    m_posPage = 0;
    m_currentPage = NULL;
  }

  /**
   * @brief Destructor deallocates all allocated memory pages.
   */
  ~ListLitAllocator() {
    for (auto page : m_pages) delete[] page;
  }

  /**
   * @brief Frees memory by marking a previously allocated array as reusable.
   *
   * @param array Pointer to the memory block being freed.
   * @param size The size of the memory block.
   */
  inline void freeMemory(Lit *array, unsigned size) {
    while (m_availaible.size() <= size)
      m_availaible.push_back(std::vector<Lit *>());
    m_availaible[size].push_back(array);
  }

  /**
   * @brief Allocates memory for a list of literals.
   *
   * If a reusable memory block of the requested size is available, it is
   * returned. Otherwise, new memory is allocated from the current page.
   * If there is not enough space, a new page is created.
   *
   * @param size The number of literals needed.
   * @return A pointer to the allocated memory.
   */
  inline Lit *askMemory(unsigned size) {
    assert(size < SIZE_PAGE_LIST_LIT);
    Lit *ret = NULL;

    // Check if there's an available reusable block
    if (size < m_availaible.size() && !m_availaible[size].empty()) {
      ret = m_availaible[size].back();
      m_availaible[size].pop_back();
    } else {
      // Allocate a new memory block if necessary
      if (!m_currentPage || m_posPage + size > SIZE_PAGE_LIST_LIT) {
        if (m_currentPage)
          freeMemory(&m_currentPage[m_posPage], SIZE_PAGE_LIST_LIT - m_posPage);

        m_currentPage = new Lit[SIZE_PAGE_LIST_LIT];
        m_pages.push_back(m_currentPage);
        m_posPage = 0;
      }

      ret = &m_currentPage[m_posPage];
      m_posPage += size;
    }

    return ret;
  }
};

class ListLit {
 private:
  int m_size = 0;
  Lit *m_array = NULL;
  ListLitAllocator *m_listLitAllocator;

 public:
  ListLit();
  void setListLit(const Lit *tab, int size, ListLitAllocator *litListAllocator);
  virtual ~ListLit();

  inline unsigned size() { return m_size; }
  inline void setSize(int size) { m_size = size; };
  inline void setArray(Lit *array) { m_array = array; }
  inline Lit &operator[](int index) {
    assert(index < m_size);
    return m_array[index];
  }
};

class BranchingHeuristic {
 protected:
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  FormulaManager *m_specs;
  ProblemManager *m_problem;
  std::vector<bool> m_isDecisionVariable;
  unsigned m_freqDecay;
  unsigned m_nbCall;
  ListLitAllocator *m_listLitAllocator = NULL;

 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristic() = delete;

  /**
   * @brief Constructs a new Branching Heuristic object.
   *
   * This constructor initializes a branching heuristic, which is responsible
   * for guiding the decision-making process during formula solving. It takes
   * into account various parameters, including problem specifications,
   * real-time formula information, activity levels, and polarity management.
   *
   * @param[in] options The configuration options for the branching heuristic.
   * @param[in] problem A pointer to the problem manager, which handles
   * problem-specific data and operations.
   * @param[in] specs A pointer to the formula manager, providing real-time
   * information about the CNF formula.
   * @param[in] activityManager A reference to the activity manager, which
   * tracks variable activity levels.
   * @param[in] polarityManager A reference to the polarity manager, which
   * handles polarity selection strategies.
   * @param[in] out The output stream where debugging or status information is
   * printed.
   */
  BranchingHeuristic(const OptionBranchingHeuristic &options,
                     ProblemManager *problem, FormulaManager *specs,
                     ActivityManager &activityManager,
                     PolarityManager &polarityManager, std::ostream &out);

  /**
   * @brief Destroy the Branching Heuristic object.
   */
  virtual ~BranchingHeuristic();

  /**
   * @brief Factory method for creating a branching heuristic instance.
   *
   * This static factory function constructs and returns a branching heuristic
   * based on the provided options. The created heuristic is responsible for
   * guiding variable selection during the search process in SAT solving,
   * potentially using strategies such as VSADS or VSIDS.
   *
   * @param[in] options The configuration options specifying the type of
   * branching heuristic to use.
   * @param[in] problem A pointer to the problem manager, which handles
   * problem-specific data and operations.
   * @param[in] specs A pointer to the formula manager, providing real-time
   * information about the CNF formula.
   * @param[in] activityManager A reference to the activity manager, which
   * tracks variable activity levels for decision-making.
   * @param[in] polarityManager A reference to the polarity manager, responsible
   * for determining variable polarities.
   * @param[in] out The output stream for logging or debugging information.
   *
   * @return A pointer to a newly created branching heuristic that matches the
   * specified options.
   */
  static BranchingHeuristic *makeBranchingHeuristic(
      const OptionBranchingHeuristic &options, ProblemManager *problem,
      FormulaManager *specs, ActivityManager &activityManager,
      PolarityManager &polarityManager, std::ostream &out);

  /**
   * @brief Select a list of literals we want to branch on it in a
   * deterministic way.
   *
   * @param vars is the set of variables under consideration.
   * @param[out] lits is the place where are stored the literals we are
   * considering.
   */
  virtual void selectLitSet(std::vector<Var> &vars, ListLit &lits) = 0;

  /**
   * @brief This function is called in order to update the branching heuristic
   * regarding the current formula and a set of variables.
   *
   * @param vars is the set of variables under consideration.
   */
  virtual void updateHeuristic(std::vector<Var> &vars) { ; }
};
}  // namespace d4