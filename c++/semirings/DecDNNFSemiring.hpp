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

#include <boost/multiprecision/gmp.hpp>
#include <map>
#include <string>
#include <vector>

#include "src/problem/ProblemTypes.hpp"

namespace semiring {

// Your namespace alias
namespace mpz = boost::multiprecision;

enum TypeNode { NODE_TOP = 0, NODE_BOT = 1, NODE_AND = 2, NODE_OR = 3 };

struct NodeStruct {
  unsigned typeNode : 4;    // for the future.
  unsigned page : 10;       // which page in m_pages.
  unsigned nbSons : 20;     // we suppose we cannot have more that 1<<20 sons.
  unsigned posInPage : 30;  // the position in the page.
};

struct Sons {
  unsigned size;
  Node sons[0];
};

struct Edge {
  unsigned idEdge;
  Node target;
};

typedef uint64_t Node;

class DecDNNFSemiring {
 private:
  const unsigned REALLOC_SIZE_NODE_INFO = 1 << 20;
  const unsigned SIZE_PAGE = 1 << 30;

  uint8_t** m_pages = NULL;
  unsigned m_nbPages = 0;
  unsigned m_positionInPage = SIZE_PAGE;

  NodeStruct* m_nodeInfo = NULL;
  unsigned m_nodeInfoCapacity = 0;
  unsigned m_idCurrentNode = 2;  // 0 for FALSE nodes, 1 for TRUE nodes.

  /**
   * @brief TODO
   */
  Node askNewNode(TypeNode type, unsigned nbChlidren) {
    if (m_idCurrentNode >= m_nodeInfoCapacity) {
      m_nodeInfoCapacity += REALLOC_SIZE_NODE_INFO;
      m_nodeInfo = (NodeStruct*)realloc(
          m_nodeInfo, sizeof(NodeStruct) * m_nodeInfoCapacity);
    }

    unsigned neededMem =
        sizeof(Edge) * (1 + ((type != NODE_AND) ? 1 : 0)) * nbChlidren;

    if (m_positionInPage + neededMem > SIZE_PAGE) {
      m_nbPages++;
      m_pages = (uint8_t**)realloc(m_pages, m_nbPages * sizeof(uint8_t*));
      m_pages[m_nbPages - 1] = new uint8_t[SIZE_PAGE];
      m_positionInPage = 0;
    }

    m_nodeInfo[m_idCurrentNode] = {
        .typeNode = (unsigned)type,
        .page = m_nbPages - 1,
        .nbSons = 0,
        .posInPage = m_positionInPage,
    };
    m_positionInPage += neededMem;

    return m_idCurrentNode++;
  }  // askNewNode

 public:
  // Required by std::default_initializable
  DecDNNFSemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  DecDNNFSemiring(unsigned nbVar,
                  const std::map<d4::Lit, std::string>& literalWeights) {}

  // --- In-Place Multiplication ---
  Node& mul(Node& a, const Node& b) const {
    // TODO
    a *= b;
    return a;
  }

  // --- In-Place Standard Binary Add ---
  Node& add(Node& a, const Node& b) {
    // TODO
    a += b;
    return a;
  }

  // --- In-Place Binary Add with Smoothing ---
  Node& add(Node& a, const Node& b, const std::vector<d4::Lit>& units) {
    // TODO
    a += b;
    return a;
  }

  Node& add(Node& a, const Node& b, const std::vector<d4::Var>& free_vars) {
    // TODO
    a += b * one(free_vars);
    return a;
  }

  Node& add(Node& a, const Node& b, const std::vector<d4::Lit>& units,
            const std::vector<d4::Var>& free_vars) {
    // TODO
    a += b * one(free_vars);
    return a;
  }

  // --- In-Place Unary Adds (Smoothing a single branch) ---
  Node& add(Node& a, const std::vector<d4::Lit>& units) {
    // TODO
    return a;
  }

  Node& add(Node& a, const std::vector<d4::Var>& free_vars) {
    // TODO
    a *= one(free_vars);
    return a;
  }

  Node& add(Node& a, const std::vector<d4::Lit>& units,
            const std::vector<d4::Var>& free_vars) {
    // TODO
    a *= one(free_vars);
    return a;
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  Node zero() { return 0; }

  Node one() { return 1; }

  Node one(const std::vector<d4::Lit>& units) {
    // TODO
    return 1;
  }

  Node one(const std::vector<d4::Var>& free_vars) {
    // TODO
    return 1;
  }

  Node one(const std::vector<d4::Lit>& units,
           const std::vector<d4::Var>& free_vars) {
    // TODO
    return 1;
  }

  // Presets (Required by Policy)
  Node presetSum(int nbGates) { return askNewNode(NODE_OR, nbGates); }
  Node presetMul(int nbGates) { return askNewNode(NODE_AND, nbGates); }
};
}  // namespace semiring