#pragma once
// Models a vertex of a hypergraph.
//
//////////////////////////////////////////////////////////////////////

#if !defined(CLS_VERTEX)
#define CLS_VERTEX

#include <memory>
#include <ostream>
#include <set>
#include <string>

#include "Globals.h"
#include "NamedEntity.h"

namespace dmlongo {

class Vertex : public NamedEntity {
 public:
  Vertex(uint id, const std::string& name) : NamedEntity(id, name) {}
  Vertex(const std::string& name);
  virtual ~Vertex();

  void setAllLabels(int label = 0) const { setLabel(0); }

  friend std::ostream& operator<<(std::ostream& out, const Vertex& v);
};

using VertexSharedPtr = std::shared_ptr<Vertex>;
using VertexSet = std::unordered_set<VertexSharedPtr, NamedEntityHash>;
using VertexVector = std::vector<VertexSharedPtr>;

// Outputs a set of vertices
std::ostream& operator<<(std::ostream& out, const VertexSet& v);

using set_type = std::set<std::shared_ptr<Vertex>>;
using powerset_type = std::set<set_type>;
powerset_type powerset(set_type const& set);

}  // namespace dmlongo

#endif
