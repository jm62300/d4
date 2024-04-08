#pragma once

#include <set>
#include <vector>

#include "CombinationIterator.h"
#include "Hyperedge.h"
#include "Hypergraph.h"
#include "Separator.h"
#include "Subedges.h"
#include "Vertex.h"

namespace dmlongo {

class SubedgeSeparatorFactory {
  std::vector<HyperedgeVector> MySubSets;
  std::vector<int> MyState;
  bool MyInit{false};

 public:
  SubedgeSeparatorFactory();
  ~SubedgeSeparatorFactory();

  void init(const HypergraphSharedPtr &hg, const HyperedgeVector &comp,
            const SeparatorSharedPtr &sep,
            const std::unique_ptr<Subedges> &subs);
  SeparatorSharedPtr next();
};

}  // namespace dmlongo
