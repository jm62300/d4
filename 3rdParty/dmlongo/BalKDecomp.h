// Models the algorithm det-k-decomp.
//
//////////////////////////////////////////////////////////////////////

#if !defined(CLS_BalKDecomp)
#define CLS_BalKDecomp

#include <unordered_map>

#include "Decomp.h"
#include "Globals.h"

namespace dmlongo {

class Hypertree;
class Subedges;

class BalKDecomp : public Decomp {
 private:
  static std::list<HypergraphSharedPtr> sFailedHg;
  static std::unordered_map<HypergraphSharedPtr, HypertreeSharedPtr> sSuccHg;

  int MyRecLevel;
  std::unique_ptr<Subedges> MySubedges;

  HypertreeSharedPtr decomp(const HyperedgeVector &HEdges);
  std::list<HypertreeSharedPtr> decompose(
      const SeparatorSharedPtr &Sep, const SuperedgeSharedPtr &Sup,
      const std::vector<DecompComponent> &Parts);

  HyperedgeVector getNeighborEdges(const HyperedgeVector &Edges) const;

  bool isBalanced(const std::vector<DecompComponent> &Parts, int CompSize);

  void expandHTree(const HypertreeSharedPtr &HTree);

  // Finds or constructs a hypergraph from a list of edges and a superedge
  // Returns false if hypergraph is in sFailedHg
  template <typename T>
  bool getHypergraph(HypergraphSharedPtr &Hg, bool &Succ, const T &Part,
                     const SuperedgeSharedPtr &Sup = nullptr) const;

 public:
  static int MyMaxRecursion;
  static HypergraphSharedPtr MyBaseGraph;

  BalKDecomp(const HypergraphSharedPtr &HGraph, int k, int RecLevel = 0);
  virtual ~BalKDecomp();

  // Constructs a hypertree decomposition of width at most MyK (if it exists)
  virtual HypertreeSharedPtr buildHypertree();

  static void init(const HypergraphSharedPtr &BaseGraph, int MaxRecursion = 0) {
    MyMaxRecursion = MaxRecursion;
    MyBaseGraph = BaseGraph;
  }
};
}  // namespace dmlongo

#endif
