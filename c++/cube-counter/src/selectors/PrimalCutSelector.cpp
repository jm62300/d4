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

#include "PrimalCutSelector.hpp"

#include <algorithm>
#include <cstdlib>
#include <vector>

#include "src/partitioner/PartitionerManager.hpp"
#include "src/representation/hypergraph/HyperEdge.hpp"
#include "src/representation/hypergraph/HyperGraph.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"

namespace d4 {

/**
 * @brief Build the primal hypergraph for a clause subset and partition it.
 *
 * Primal hypergraph: nodes = variables, hyperedges = clauses.
 * PaToH partitions the variable nodes into two balanced groups.
 * A clause is a "cut hyperedge" if it contains variables from both groups.
 */
void PrimalCutSelector::computeCut(const parser::Formula& formula,
                                   const std::vector<unsigned>& clauseIndices,
                                   std::vector<unsigned>& cutClauses,
                                   std::vector<unsigned>& part0,
                                   std::vector<unsigned>& part1) {
  if (clauseIndices.size() < 10) {
    // Too small to partition meaningfully; treat everything as cut.
    cutClauses.insert(cutClauses.end(), clauseIndices.begin(),
                      clauseIndices.end());
    return;
  }

  // --- Build primal hypergraph ---
  // Compute upper bound on flat buffer size.
  unsigned sumSizes = 0;
  for (unsigned idx : clauseIndices) sumSizes += formula.clauses[idx].size();

  // Each HyperEdge occupies sizeof(HyperEdge) + size*sizeof(unsigned) bytes
  // (flexible array member; placement new into flat buffer).
  std::vector<char> buffer(clauseIndices.size() * sizeof(HyperEdge) +
                           sumSizes * sizeof(unsigned));
  size_t pos = 0;

  InfoHyperGraph info{(unsigned)clauseIndices.size(), formula.nbVar + 1,
                      sumSizes};
  HyperGraph hypergraph((unsigned)clauseIndices.size());

  std::vector<unsigned> varBuf;
  for (unsigned i = 0; i < clauseIndices.size(); i++) {
    unsigned clauseIdx = clauseIndices[i];
    const auto& cl = formula.clauses[clauseIdx];

    varBuf.clear();
    for (auto l : cl) varBuf.push_back((unsigned)std::abs(l));
    std::sort(varBuf.begin(), varBuf.end());
    varBuf.erase(std::unique(varBuf.begin(), varBuf.end()), varBuf.end());

    // Placement-new into flat buffer; HyperEdge id = original clause index.
    HyperEdge* edge = new (&buffer[pos])
        HyperEdge(clauseIdx, (unsigned)varBuf.size(), varBuf.data());
    hypergraph.addEdge(edge);
    pos += sizeof(HyperEdge) + varBuf.size() * sizeof(unsigned);
  }

  // --- Run PaToH bisection ---
  auto* partitioner = PartitionerManager::makePartitioner(PARTITIONER_PATOH,
                                                          info, m_out);
  std::vector<int> partition;
  partitioner->computePartition(hypergraph, PartitionerManager::NORMAL,
                                partition);
  delete partitioner;

  // --- Classify clauses ---
  for (unsigned clauseIdx : clauseIndices) {
    const auto& cl = formula.clauses[clauseIdx];
    bool hasPart0 = false, hasPart1 = false;
    for (auto l : cl) {
      unsigned v = (unsigned)std::abs(l);
      if (v < partition.size()) {
        if (partition[v] == 0) hasPart0 = true;
        else hasPart1 = true;
      }
    }
    if (hasPart0 && hasPart1)
      cutClauses.push_back(clauseIdx);
    else if (hasPart0)
      part0.push_back(clauseIdx);
    else
      part1.push_back(clauseIdx);
  }

  // Degenerate partition: put everything in cut so nothing is lost.
  if (part0.empty() || part1.empty()) {
    cutClauses.insert(cutClauses.end(), part0.begin(), part0.end());
    cutClauses.insert(cutClauses.end(), part1.begin(), part1.end());
    part0.clear();
    part1.clear();
  }
}

std::vector<unsigned> PrimalCutSelector::select(
    const parser::Formula& formula) {
  std::vector<unsigned> allClauses;
  allClauses.reserve(formula.clauses.size());
  for (unsigned i = 0; i < formula.clauses.size(); i++) allClauses.push_back(i);

  std::vector<unsigned> cutClauses, part0, part1;
  computeCut(formula, allClauses, cutClauses, part0, part1);

  m_out << "c [PRIMAL-CUT] cut=" << cutClauses.size()
        << " part0=" << part0.size() << " part1=" << part1.size() << '\n';

  return cutClauses;
}

}  // namespace d4
