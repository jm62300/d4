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

namespace mpz = boost::multiprecision;

namespace semiring {

/**
 * @brief Represents the logical type of a Node in the Dec-DNNF graph.
 */
enum TypeNode { NODE_TOP = 0, NODE_BOT = 1, NODE_AND = 2, NODE_OR = 3 };

/**
 * @brief Highly optimized memory layout for node metadata.
 * Fits exactly into 64 bits (8 bytes) to minimize cache misses.
 */
struct NodeStruct {
  unsigned typeNode : 4;  // The TypeNode (AND, OR, TOP, BOT).
  unsigned page
      : 10;  // The index of the memory page in m_pages (max 1024 pages).
  unsigned nbSons
      : 20;  // Current number of allocated children (max ~1 million).
  unsigned posInPage : 30;  // Byte offset within the specific 1GB memory page.
};

/**
 * @brief Metadata for an edge pointing to where its specific data
 * (units and free variables) is stored in the paged memory.
 */
struct EdgeInfo {
  unsigned page;
  unsigned posInPage;
};

// Typedef for node identifiers to keep signatures clean.
typedef unsigned Node;

/**
 * @brief Represents a directed edge connecting an OR node to its children.
 */
struct Edge {
  unsigned idEdge;  // Unique identifier linking to EdgeInfo.
  Node target;      // The destination node this edge points to.
};

/**
 * @brief A custom Semiring designed to construct and manage a Dec-DNNF graph.
 *
 * This class uses a custom paged-memory allocator to drastically speed up
 * node and edge allocations and reduce memory fragmentation. Instead of
 * standard `new`/`malloc` for each node, it pre-allocates massive 1GB "pages"
 * and sequentially assigns pointers within them.
 */
class DecDNNFSemiring {
 private:
  static const unsigned REALLOC_SIZE_NODE_INFO = 1
                                                 << 20;  // 1M nodes per realloc
  static const unsigned REALLOC_SIZE_EDGE_INFO = 1
                                                 << 20;  // 1M edges per realloc
  static const unsigned SIZE_PAGE = 1 << 30;             // 1GB per memory page

  uint8_t** m_pages = NULL;  // Array of pointers to 1GB memory pages.
  unsigned m_nbPages = 0;    // Current number of allocated pages.
  unsigned m_positionInPage = SIZE_PAGE;  // Offset in the current page (starts
                                          // full to force initial alloc).

  NodeStruct* m_nodeInfo = NULL;    // Metadata for all nodes.
  unsigned m_nodeInfoCapacity = 0;  // Current capacity of the m_nodeInfo array.
  unsigned m_idCurrentNode = 2;     // Next available (0=BOT, 1=TOP reserved).

  EdgeInfo* m_edgeInfo = NULL;      // Metadata for all edges.
  unsigned m_edgeInfoCapacity = 0;  // Current capacity of the m_edgeInfo array.
  unsigned m_idEdge = 0;            // Next available Edge ID.

  /**
   * @brief Allocates a new node and reserves memory in the current page for its
   * children.
   *
   * @param type The type of the node (AND / OR).
   * @param nbChildren The number of children this node will have.
   * @return Node The unique ID of the newly created node.
   */
  Node askNewNode(TypeNode type, unsigned nbChildren) {
    if (m_idCurrentNode >= m_nodeInfoCapacity) {
      m_nodeInfoCapacity += REALLOC_SIZE_NODE_INFO;
      m_nodeInfo = (NodeStruct*)realloc(
          m_nodeInfo, sizeof(NodeStruct) * m_nodeInfoCapacity);
    }

    // AND nodes store an array of Node IDs. OR nodes store an array of Edge
    // structs.
    unsigned neededMem =
        nbChildren * ((type == NODE_AND) ? sizeof(Node) : sizeof(Edge));

    // If the current page is full, allocate a new 1GB page.
    if (m_positionInPage + neededMem > SIZE_PAGE) {
      m_nbPages++;
      m_pages = (uint8_t**)realloc(m_pages, m_nbPages * sizeof(uint8_t*));
      m_pages[m_nbPages - 1] = new uint8_t[SIZE_PAGE];
      m_positionInPage = 0;
    }

    // Register node metadata
    m_nodeInfo[m_idCurrentNode] = {
        .typeNode = (unsigned)type,
        .page = m_nbPages - 1,
        .nbSons = 0,  // Starts at 0, incremented via add/mul
        .posInPage = m_positionInPage,
    };

    // Advance the allocator bump pointer
    m_positionInPage += neededMem;

    return m_idCurrentNode++;
  }  // askNewNode

  /**
   * @brief Allocates a new edge and stores its associated unit and free
   * variables.
   *
   * @param units The unit literals propagated on this edge.
   * @param free_vars Variables that become free (unassigned) on this branch.
   * @return unsigned The unique ID of the newly created edge.
   */
  unsigned askNewEdge(const std::vector<d4::Lit>& units,
                      const std::vector<d4::Var>& free_vars) {
    if (m_idEdge >= m_edgeInfoCapacity) {
      m_edgeInfoCapacity += REALLOC_SIZE_EDGE_INFO;
      m_edgeInfo =
          (EdgeInfo*)realloc(m_edgeInfo, sizeof(EdgeInfo) * m_edgeInfoCapacity);
    }

    // Memory layout: [units...] 0 [free_vars...] 0
    // The '+2' accounts for the '0' separators.
    unsigned neededMem =
        (2 + units.size() + free_vars.size()) * sizeof(unsigned);

    if (m_positionInPage + neededMem > SIZE_PAGE) {
      m_nbPages++;
      m_pages = (uint8_t**)realloc(m_pages, m_nbPages * sizeof(uint8_t*));
      m_pages[m_nbPages - 1] = new uint8_t[SIZE_PAGE];
      m_positionInPage = 0;
    }

    // Register edge metadata
    m_edgeInfo[m_idEdge].page = m_nbPages - 1;
    m_edgeInfo[m_idEdge].posInPage = m_positionInPage;

    // Get pointer to the allocated block and write the data
    unsigned* tab = (unsigned*)(m_pages[m_nbPages - 1] + m_positionInPage);
    unsigned i = 0;

    for (auto& l : units) tab[i++] = l.intern();
    tab[i++] = 0;  // Separator
    for (auto& v : free_vars) tab[i++] = v;

    m_positionInPage += neededMem;
    return m_idEdge++;
  }  // askNewEdge

 public:
  // Required by std::default_initializable
  DecDNNFSemiring() = default;

  // Required by the SemiringPolicy constructor constraint
  DecDNNFSemiring(unsigned nbVar,
                  const std::map<d4::Lit, std::string>& literalWeights) {}

  /**
   * @brief Connects a child node to an AND node.
   * @param a The parent AND node.
   * @param b The child node to append.
   * @return Node& Reference to the parent node `a`.
   */
  Node& mul(Node& a, const Node& b) {
    NodeStruct& n = m_nodeInfo[a];

    // Retrieve the pointer to the children array
    Node* sons = (Node*)(m_pages[n.page] + n.posInPage);

    // Append the child and increment the son counter
    sons[n.nbSons++] = b;
    return a;
  }  // mul

  /**
   * @brief Connects a child node to an OR node via a new edge carrying
   * variables.
   * @param a The parent OR node.
   * @param b The target child node.
   * @param units Unit literals associated with this edge.
   * @param free_vars Free variables associated with this edge.
   * @return Node& Reference to the parent node `a`.
   */
  Node& add(Node& a, const Node& b, const std::vector<d4::Lit>& units,
            const std::vector<d4::Var>& free_vars) {
    NodeStruct& n = m_nodeInfo[a];

    // Retrieve the pointer to the edge array
    Edge* sons = (Edge*)(m_pages[n.page] + n.posInPage);

    // Create the edge and link it
    sons[n.nbSons].target = b;
    sons[n.nbSons].idEdge = askNewEdge(units, free_vars);
    n.nbSons++;

    return a;
  }  // add

  /**
   * @brief Returns the universal constant node for FALSE (Bottom).
   * @return Node Identifier for the BOT node (0).
   */
  Node zero() { return 0; }

  /**
   * @brief Returns the universal constant node for TRUE (Top).
   * @return Node Identifier for the TOP node (1).
   */
  Node one() { return 1; }

  /**
   * @brief Pre-allocates an OR node with a known number of children.
   * @param nbGates The total number of edges this OR node will have.
   * @return Node The identifier of the newly created OR node.
   */
  Node presetSum(int nbGates) { return askNewNode(NODE_OR, nbGates); }

  /**
   * @brief Pre-allocates an AND node with a known number of children.
   * @param nbGates The total number of nodes this AND node will point to.
   * @return Node The identifier of the newly created AND node.
   */
  Node presetMul(int nbGates) { return askNewNode(NODE_AND, nbGates); }

  /**
   * @brief Computes the weighted model count of the Dec-DNNF structure.
   * @tparam T The numeric type (e.g., double, multiprecision integer).
   * @param n The root node of the graph to evaluate.
   * @param assums Unit assumptions to restrict the counting space.
   * @param weight The array of literal weights (size must be 2 * nbVar + 2).
   * @param nbVar The total number of variables in the formula.
   * @return The total weighted model count under the given assumptions.
   */
  template <class T>
  T count(Node n, const std::vector<d4::Lit>& assums,
          const std::vector<T>& weight, unsigned nbVar) const {
    enum Status { NOT_PROCESS, IN_PROCESS, DONE };
    std::vector<Status> status(m_idCurrentNode, NOT_PROCESS);
    std::vector<T> subCount(m_idCurrentNode, T(0));

    // Set the assumptions. 2 means unassigned.
    std::vector<uint8_t> valuation(nbVar + 1, 2);
    for (auto& l : assums) valuation[l.var()] = l.sign();

    // Base cases
    subCount[0] = T(0);
    subCount[1] = T(1);
    status[0] = DONE;
    status[1] = DONE;

    std::vector<Node> stack;
    stack.push_back(n);

    while (!stack.empty()) {
      Node current = stack.back();
      NodeStruct& info = m_nodeInfo[current];

      switch (status[current]) {
        case NOT_PROCESS: {
          // 1. Mark as IN_PROCESS so we calculate it AFTER its children finish.
          // Note: We DO NOT pop the stack here.
          status[current] = IN_PROCESS;

          uint8_t* ptr = m_pages[info.page] + info.posInPage;
          for (unsigned i = 0; i < info.nbSons; i++) {
            Node s = 0;
            if (info.typeNode == NODE_AND) {
              s = *((Node*)ptr);
              ptr += sizeof(Node);
            } else {
              assert(info.typeNode == NODE_OR);
              s = ((Edge*)ptr)->target;
              ptr += sizeof(Edge);
            }

            if (status[s] == NOT_PROCESS) {
              stack.push_back(s);
            }
          }
          break;
        }

        case IN_PROCESS: {
          T& computedValue = subCount[current];
          uint8_t* ptr = m_pages[info.page] + info.posInPage;

          if (info.typeNode == NODE_AND) {
            // AND node multiplicative identity starts at 1, not 0!
            computedValue = T(1);
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              assert(status[s] == DONE);
              computedValue *= subCount[s];
            }
          } else {
            // OR node additive identity starts at 0
            computedValue = T(0);
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);
              assert(status[e.target] == DONE);

              EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
              unsigned* tab =
                  (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);
              T multiplier = T(1);

              // Get the score for the unit literals.
              while (*tab != 0) {
                d4::Lit l = d4::Lit::makeLit((*tab) >> 1, (*tab) & 1);
                tab++;

                if (valuation[l.var()] > 1) {
                  multiplier *= weight[l.intern()];
                } else if (valuation[l.var()] != l.sign()) {
                  multiplier = T(0);
                  break;
                }
              }

              tab++;  // Skip the '0' separator
              if (multiplier == T(0)) continue;

              // Get the score for the free variables.
              while (*tab != 0) {
                d4::Var v = *tab;
                tab++;

                if (valuation[v] > 1) {
                  multiplier *= weight[v << 1] + weight[(v << 1) | 1];
                } else {
                  multiplier *= weight[(v << 1) | valuation[v]];
                }
              }

              computedValue += subCount[e.target] * multiplier;
            }
          }

          // 6. Node is finished computing. Pop it and mark DONE.
          status[current] = DONE;
          stack.pop_back();
          break;
        }

        case DONE:
          // Node was already fully computed by an earlier branch, just discard
          // it.
          stack.pop_back();
          break;
      }
    }

    return subCount[n];
  }  // count

  inline unsigned getNbNodes() const { return m_idCurrentNode; }
  inline unsigned getNbEdges() const { return m_idEdge; }
};
}  // namespace semiring