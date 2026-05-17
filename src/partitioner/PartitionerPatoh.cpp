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
#include "PartitionerPatoh.hpp"

#include <iostream>
#include <vector>

#include "3rdParty/patoh/patoh.h"
#include "src/exceptions/OptionException.hpp"

namespace d4 {

/**
 * @brief PartitionerPatoh::PartitionerPatoh implementation.
 */
PartitionerPatoh::PartitionerPatoh(const InfoHyperGraph &infoHyperGraph,
                                   std::ostream &out) {
  // allocate the memory
  m_pins = new int[infoHyperGraph.sumEdgeSizes];
  m_partweights = new int[2];
  m_xpins = new int[(infoHyperGraph.maxNbEdges + 3)];
  m_partvec = new int[(infoHyperGraph.maxNbNodes + 3)];
  m_cwghts = new int[(infoHyperGraph.maxNbNodes + 3)];

  // set all weight to 1
  for (unsigned i = 0; i < (infoHyperGraph.maxNbNodes + 3); i++)
    m_cwghts[i] = 1;

  m_mapNodes.resize(infoHyperGraph.maxNbNodes + 3, false);
  m_markedNodes.resize(infoHyperGraph.maxNbNodes + 3, false);
}  // constructor

/**
   Destructor.
 */
PartitionerPatoh::~PartitionerPatoh() {
  delete[] m_pins;
  delete[] m_partweights;
  delete[] m_xpins;
  delete[] m_partvec;
  delete[] m_cwghts;
}  // destructor

/**
 * @brief Get a partition from the hypergraph.
 *
 * @param[in] hypergraph is the graph we search for a partition.
 * @param[out] parition is the resulting partition.
 */
void PartitionerPatoh::computePartition(HyperGraph &hypergraph, Level level,
                                        std::vector<int> &partition) {
  std::vector<unsigned> elts;

  // graph initialization and shift the hypergraph
  unsigned sizeXpins = 0;
  int posPins = 0;

  for (unsigned i = 0; i < hypergraph.getNbEdges(); i++) {
    m_xpins[sizeXpins++] = posPins;
    for (unsigned j = 0; j < hypergraph[i].getSize(); j++) {
      unsigned x = hypergraph[i][j];
      assert(x < m_markedNodes.size());
      if (!m_markedNodes[x]) {
        m_markedNodes[x] = true;
        m_mapNodes[x] = elts.size();
        elts.push_back(x);
      }

      m_pins[posPins++] = m_mapNodes[x];
    }
  }

  unsigned maxVal = 0;
  for (auto &x : elts) {
    m_markedNodes[x] = false;
    if (x > maxVal) maxVal = x;
  }
  m_xpins[sizeXpins] = posPins;

  // hypergraph partitioner
  PaToH_Parameters args;
  switch (level) {
    case NORMAL:
      PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_DEFAULT);
      break;
    case SPEED:
      PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_SPEED);
      break;
    case QUALITY:
      PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_QUALITY);
      break;
    default:
      throw(OptionException("Wrong option given to the partioner.", __FILE__,
                            __LINE__));
  }

  args._k = 2;
  args.seed = 2911;

  int cut;
  PaToH_Alloc(&args, elts.size(), sizeXpins, 1, m_cwghts, NULL, m_xpins,
              m_pins);
  PaToH_Part(&args, elts.size(), sizeXpins, 1, 0, m_cwghts, NULL, m_xpins,
             m_pins, NULL, m_partvec, m_partweights, &cut);

  partition.resize(maxVal + 1);
  for (unsigned i = 0; i < elts.size(); i++) partition[elts[i]] = m_partvec[i];
  PaToH_Free();
}  // computePartition

}  // namespace d4
