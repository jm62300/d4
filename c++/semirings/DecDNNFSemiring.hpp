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

#include <map>
#include <string>
#include <vector>

#include "src/problem/ProblemTypes.hpp"
#include "src/utils/MpzTypes.hpp"

namespace mpz = d4MpzTypes;

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
    tab[i++] = 0;  // Terminator

    m_positionInPage += neededMem;
    return m_idEdge++;
  }  // askNewEdge

 public:
  // Required by std::default_initializable
  DecDNNFSemiring() = default;

  // Required by the SemiringPolicy constructor constraint
  DecDNNFSemiring(unsigned nbVar,
                  const std::map<d4::Lit, std::string>& literalWeights) {}

  ~DecDNNFSemiring() {
    for (unsigned i = 0; i < m_nbPages; i++) delete[] m_pages[i];
    free(m_pages);
    free(m_nodeInfo);
    free(m_edgeInfo);
  }

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
  static Node zero() { return 0; }

  /**
   * @brief Returns the universal constant node for TRUE (Top).
   * @return Node Identifier for the TOP node (1).
   */
  static Node one() { return 1; }

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

  /**
   * @brief Prints the Dec-DNNF backbone structure to a stream.
   * Dynamically expands edges with units into virtual AND nodes to maintain
   * a strictly valid NNF graph topology.
   *
   * @param n The root node of the graph to print.
   * @param out The output stream.
   */
  void printNNF(Node n, std::ostream& out) const {
    enum Status { NOT_PROCESS, IN_PROCESS, DONE };
    std::vector<Status> status(m_idCurrentNode, NOT_PROCESS);
    std::vector<unsigned> nodeIdx(m_idCurrentNode, 0);
    unsigned idx = 1;

    std::vector<Node> stack;
    stack.push_back(n);

    while (!stack.empty()) {
      Node current = stack.back();

      // Handle terminal TOP / BOT nodes natively
      if (current == 0 || current == 1) {
        if (status[current] == NOT_PROCESS) {
          nodeIdx[current] = idx++;
          out << (current == 0 ? "f " : "t ") << nodeIdx[current] << " 0\n";
          status[current] = DONE;
        }
        stack.pop_back();
        continue;
      }

      NodeStruct& info = m_nodeInfo[current];

      switch (status[current]) {
        case NOT_PROCESS: {
          status[current] = IN_PROCESS;
          nodeIdx[current] = idx++;

          if (info.typeNode == NODE_AND)
            out << "a " << nodeIdx[current] << " 0\n";
          else if (info.typeNode == NODE_OR)
            out << "o " << nodeIdx[current] << " 0\n";

          uint8_t* ptr = m_pages[info.page] + info.posInPage;
          if (info.typeNode == NODE_AND) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (status[s] == NOT_PROCESS) stack.push_back(s);
            }
          } else if (info.typeNode == NODE_OR) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node target = ((Edge*)ptr)->target;
              ptr += sizeof(Edge);
              if (status[target] == NOT_PROCESS) stack.push_back(target);
            }
          }
          break;
        }

        case IN_PROCESS: {
          status[current] = DONE;
          unsigned myIdx = nodeIdx[current];
          uint8_t* ptr = m_pages[info.page] + info.posInPage;

          if (info.typeNode == NODE_AND) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              out << myIdx << " " << nodeIdx[s] << " 0\n";
            }
          } else if (info.typeNode == NODE_OR) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);

              EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
              unsigned* tab =
                  (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);

              // Print the parent-child relationship
              out << myIdx << ' ' << nodeIdx[e.target] << ' ';

              // Print the unit literals directly on the edge
              while (*tab != 0) {
                out << (((*tab) & 1) ? "-" : "") << ((*tab) >> 1) << ' ';
                tab++;
              }
              out << "0\n";

              // Note: If you ever need to print the free variables in the
              // future, you would just do tab++; here to skip the 0 separator,
              // and then run another while(*tab != 0) loop!
            }
          }

          stack.pop_back();
          break;
        }

        case DONE:
          stack.pop_back();
          break;
      }
    }
  }  // count

  /**
   * @brief Checks if the Dec-DNNF is satisfiable under a given set of
   * assumptions.
   * @param n The root node of the graph to check.
   * @param assums Unit assumptions that must hold true.
   * @param nbVar The total number of variables in the formula.
   * @return true if there is at least one valid model, false otherwise.
   */
  bool isSAT(Node n, const std::vector<d4::Lit>& assums, unsigned nbVar) const {
    // Valuation mapping: 2 = unassigned. Otherwise, it matches the literal's
    // required sign (0 for True, 1 for False).
    std::vector<uint8_t> valuation(nbVar + 1, 2);
    for (auto& l : assums) valuation[l.var()] = l.sign();

    enum Status { NOT_PROCESS, IN_PROCESS, DONE };
    std::vector<Status> status(m_idCurrentNode, NOT_PROCESS);
    std::vector<int8_t> subSat(m_idCurrentNode, -1);  // -1=unk, 0=uns, 1=sat

    // Base cases
    subSat[0] = 0;  // BOT is always false
    subSat[1] = 1;  // TOP is always true
    status[0] = DONE;
    status[1] = DONE;

    std::vector<Node> stack;
    stack.push_back(n);

    while (!stack.empty()) {
      Node current = stack.back();
      NodeStruct& info = m_nodeInfo[current];

      switch (status[current]) {
        case NOT_PROCESS: {
          status[current] = IN_PROCESS;
          uint8_t* ptr = m_pages[info.page] + info.posInPage;

          if (info.typeNode == NODE_AND) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (status[s] == NOT_PROCESS) stack.push_back(s);
            }
          } else if (info.typeNode == NODE_OR) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);

              // PRE-PRUNING: Check if this branch's units conflict with
              // assumptions
              EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
              unsigned* tab =
                  (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);

              bool edgeConflict = false;
              while (*tab != 0) {
                d4::Lit l = d4::Lit::makeLit((*tab) >> 1, (*tab) & 1);
                tab++;
                // If a variable is assigned (!= 2) and its sign doesn't match
                // the literal -> Conflict
                if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
                  edgeConflict = true;
                  break;
                }
              }

              // Only explore the child if the edge itself doesn't conflict!
              if (!edgeConflict && status[e.target] == NOT_PROCESS) {
                stack.push_back(e.target);
              }
            }
          }
          break;
        }

        case IN_PROCESS: {
          status[current] = DONE;
          uint8_t* ptr = m_pages[info.page] + info.posInPage;

          if (info.typeNode == NODE_AND) {
            subSat[current] = 1;  // Assume true until proven false
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (subSat[s] == 0) {
                subSat[current] = 0;
                break;  // Short-circuit AND (one false child ruins it)
              }
            }
          } else if (info.typeNode == NODE_OR) {
            subSat[current] = 0;  // Assume false until proven true
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);

              EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
              unsigned* tab =
                  (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);

              bool edgeConflict = false;
              while (*tab != 0) {
                d4::Lit l = d4::Lit::makeLit((*tab) >> 1, (*tab) & 1);
                tab++;
                if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
                  edgeConflict = true;
                  break;
                }
              }

              // If edge is valid AND the target to SAT, the OR node is SAT
              if (!edgeConflict && subSat[e.target] == 1) {
                subSat[current] = 1;
                break;  // Short-circuit OR (one true valid branch is enough)
              }
            }
          }

          stack.pop_back();
          break;
        }

        case DONE:
          // Node was already fully computed by an earlier branch, skip it.
          stack.pop_back();
          break;
      }
    }

    return subSat[n] == 1;
  }  // isSAT

  /**
   * @brief Enumerates all disjoint partial models of the Dec-DNNF.
   * Based on "Leveraging Decision-DNNF Compilation for Enumerating Disjoint
   * Partial Models" Guarantees polynomial delay by using a two-pass
   * reachability + backtracking approach.
   *
   * @param n The root node of the graph.
   * @param assums Unit assumptions that must hold true.
   * @param nbVar The total number of variables in the formula.
   * @param onModel Callback function invoked for each valid partial model.
   */
  void enumeratePartialModels(
      Node n, const std::vector<d4::Lit>& assums, unsigned nbVar,
      const std::function<void(std::vector<d4::Lit>&)>& onModel) const {
    // 1. Prepare valuations for fast checking. (2 = unassigned)
    std::vector<uint8_t> valuation(nbVar + 1, 2);
    for (auto& l : assums) valuation[l.var()] = l.sign();

    // =========================================================================
    // PASS 1: Compute Satisfiability of all nodes under these assumptions.
    // This is strictly required to guarantee polynomial delay by pruning dead
    // branches.
    // =========================================================================
    enum Status { NOT_PROCESS, IN_PROCESS, DONE };
    std::vector<Status> status(m_idCurrentNode, NOT_PROCESS);
    std::vector<int8_t> subSat(m_idCurrentNode, -1);

    subSat[0] = 0;  // BOT is always false
    subSat[1] = 1;  // TOP is always true
    status[0] = DONE;
    status[1] = DONE;

    std::vector<Node> evalStack;
    evalStack.push_back(n);

    while (!evalStack.empty()) {
      Node current = evalStack.back();
      NodeStruct& info = m_nodeInfo[current];

      switch (status[current]) {
        case NOT_PROCESS: {
          status[current] = IN_PROCESS;
          uint8_t* ptr = m_pages[info.page] + info.posInPage;

          if (info.typeNode == NODE_AND) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (status[s] == NOT_PROCESS) evalStack.push_back(s);
            }
          } else if (info.typeNode == NODE_OR) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);

              EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
              unsigned* tab =
                  (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);

              bool edgeConflict = false;
              while (*tab != 0) {
                d4::Lit l = d4::Lit::makeLit((*tab) >> 1, (*tab) & 1);
                tab++;
                if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
                  edgeConflict = true;
                  break;
                }
              }

              if (!edgeConflict && status[e.target] == NOT_PROCESS) {
                evalStack.push_back(e.target);
              }
            }
          }
          break;
        }

        case IN_PROCESS: {
          status[current] = DONE;
          uint8_t* ptr = m_pages[info.page] + info.posInPage;

          if (info.typeNode == NODE_AND) {
            subSat[current] = 1;
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (subSat[s] == 0) {
                subSat[current] = 0;
                break;
              }
            }
          } else if (info.typeNode == NODE_OR) {
            subSat[current] = 0;
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);

              EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
              unsigned* tab =
                  (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);

              bool edgeConflict = false;
              while (*tab != 0) {
                d4::Lit l = d4::Lit::makeLit((*tab) >> 1, (*tab) & 1);
                tab++;
                if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
                  edgeConflict = true;
                  break;
                }
              }

              if (!edgeConflict && subSat[e.target] == 1) {
                subSat[current] = 1;
                break;
              }
            }
          }
          evalStack.pop_back();
          break;
        }

        case DONE:
          evalStack.pop_back();
          break;
      }
    }

    // If the root is UNSAT, there are zero models to enumerate.
    if (subSat[n] != 1) return;

    // =========================================================================
    // PASS 2: Backtracking Enumerator
    // =========================================================================
    std::vector<Node> stack;
    stack.push_back(n);

    // The current partial model being built (starts with the assumptions)
    std::vector<d4::Lit> current_pm = assums;

    // Zero-allocation backtracking lambda
    auto backtrack = [&](auto& self) -> void {
      if (stack.empty()) {
        onModel(current_pm);  // Yield the fully constructed partial model
        return;
      }

      Node current = stack.back();
      stack.pop_back();

      // Because of subSat pruning, we NEVER hit a 0 (BOT). We might hit a 1
      // (TOP).
      if (current == 1) {
        self(self);                // Process the rest of the stack
        stack.push_back(current);  // Restore state
        return;
      }

      NodeStruct& info = m_nodeInfo[current];
      uint8_t* ptr = m_pages[info.page] + info.posInPage;

      if (info.typeNode == NODE_AND) {
        // Since we know this AND node is SAT, all its children are SAT.
        // Push them in reverse so they are popped and processed left-to-right.
        for (int i = info.nbSons - 1; i >= 0; i--) {
          Node s = *((Node*)(ptr + i * sizeof(Node)));
          stack.push_back(s);
        }

        self(self);  // Dive deeper

        // Restore stack
        for (unsigned i = 0; i < info.nbSons; i++) stack.pop_back();
        stack.push_back(current);
      } else if (info.typeNode == NODE_OR) {
        for (unsigned i = 0; i < info.nbSons; i++) {
          Edge& e = *((Edge*)ptr);
          ptr += sizeof(Edge);

          // STRICT PRUNING: Only explore branches mathematically proven to be
          // SAT!
          if (subSat[e.target] != 1) continue;

          EdgeInfo& edgeInfo = m_edgeInfo[e.idEdge];
          unsigned* tab =
              (unsigned*)(m_pages[edgeInfo.page] + edgeInfo.posInPage);

          bool edgeConflict = false;
          unsigned added_lits = 0;

          // Verify edge consistency
          unsigned* scan_tab = tab;
          while (*scan_tab != 0) {
            d4::Lit l = d4::Lit::makeLit((*scan_tab) >> 1, (*scan_tab) & 1);
            scan_tab++;
            if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
              edgeConflict = true;
              break;
            }
          }

          if (!edgeConflict) {
            // Apply units to the current partial model
            while (*tab != 0) {
              d4::Lit l = d4::Lit::makeLit((*tab) >> 1, (*tab) & 1);
              tab++;
              // Only push if it wasn't already included by the global
              // assumptions
              if (valuation[l.var()] == 2) {
                current_pm.push_back(l);
                added_lits++;
              }
            }

            // Note: We intentionally ignore free variables (*tab != 0 after the
            // separator) because a partial model implies they can take any
            // assignment.

            stack.push_back(e.target);

            self(self);  // Recurse into this branch

            // Clean up dynamically added variables for the next iteration
            stack.pop_back();
            for (unsigned j = 0; j < added_lits; j++) current_pm.pop_back();
          }
        }
        stack.push_back(current);
      }
    };

    backtrack(backtrack);
  }

  inline unsigned getNbNodes() const { return m_idCurrentNode; }
  inline unsigned getNbEdges() const { return m_idEdge; }

  /**
   * @brief Build a Tseitin encoding of the AND structure of this Dec-DNNF.
   *
   * Assigns a fresh SAT variable x_n = (tseitinOffset + n) to every node n.
   * For each AND node: adds implication clauses -x_n ∨ x_ci for each child ci,
   * so that assuming x_n propagates to all children.  OR nodes get no clauses
   * (their branching is handled externally).  BOT (0) is forced false, TOP (1)
   * is forced true.
   *
   * The addClause callback receives a zero-terminated literal array in DIMACS
   * convention (positive = true, negative = false).  The caller is responsible
   * for loading these into whatever SAT solver it uses.
   *
   * @param root        Root node of the DNNF.
   * @param tsOffset    First fresh variable index: node n maps to tsOffset+n.
   * @param addClause   Called once per clause with a {lit…, 0} vector.
   */
  void buildTseitin(
      Node root, unsigned tsOffset,
      const std::function<void(const std::vector<int>&)>& addClause) const {
    // Force BOT=false, TOP=true.
    addClause({-(int)(tsOffset + 0), 0});
    addClause({(int)(tsOffset + 1), 0});

    // Traverse the DNNF (iterative DFS, each node visited once).
    std::vector<bool> visited(m_idCurrentNode, false);
    std::vector<Node> stack;
    stack.push_back(root);

    while (!stack.empty()) {
      Node cur = stack.back();
      stack.pop_back();
      if (visited[cur]) continue;
      visited[cur] = true;

      if (cur == 0 || cur == 1) continue;  // BOT/TOP already handled

      const NodeStruct& info = m_nodeInfo[cur];
      uint8_t* ptr = m_pages[info.page] + info.posInPage;
      int xCur = (int)(tsOffset + cur);

      if (info.typeNode == NODE_AND) {
        for (unsigned i = 0; i < info.nbSons; i++) {
          Node child = *((Node*)ptr);
          ptr += sizeof(Node);
          // -x_cur ∨ x_child
          addClause({-xCur, (int)(tsOffset + child), 0});
          if (!visited[child]) stack.push_back(child);
        }
      } else if (info.typeNode == NODE_OR) {
        for (unsigned i = 0; i < info.nbSons; i++) {
          Node target = ((Edge*)ptr)->target;
          ptr += sizeof(Edge);
          if (!visited[target]) stack.push_back(target);
        }
      }
    }
  }

  /**
   * @brief Like enumeratePartialModels but calls shouldPrune before recursing
   * into each OR branch.
   *
   * @param shouldPrune Called as shouldPrune(target, current_partial_model)
   *                    where current_partial_model already contains the edge
   *                    units for this branch.  Return true to skip the branch.
   * @param onModel     Called for each surviving complete partial model.
   */
  void enumeratePartialModelsWithPruning(
      Node n, const std::vector<d4::Lit>& assums, unsigned nbVar,
      const std::function<bool(Node, const std::vector<d4::Lit>&)>& shouldPrune,
      const std::function<void(std::vector<d4::Lit>&)>& onModel) const {
    std::vector<uint8_t> valuation(nbVar + 1, 2);
    for (auto& l : assums) valuation[l.var()] = l.sign();

    // Pass 1: compute subSat (identical to enumeratePartialModels).
    enum Status { NOT_PROCESS, IN_PROCESS, DONE };
    std::vector<Status> status(m_idCurrentNode, NOT_PROCESS);
    std::vector<int8_t> subSat(m_idCurrentNode, -1);
    subSat[0] = 0;
    subSat[1] = 1;
    status[0] = DONE;
    status[1] = DONE;

    std::vector<Node> evalStack;
    evalStack.push_back(n);
    while (!evalStack.empty()) {
      Node cur = evalStack.back();
      const NodeStruct& info = m_nodeInfo[cur];
      switch (status[cur]) {
        case NOT_PROCESS: {
          status[cur] = IN_PROCESS;
          uint8_t* ptr = m_pages[info.page] + info.posInPage;
          if (info.typeNode == NODE_AND) {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (status[s] == NOT_PROCESS) evalStack.push_back(s);
            }
          } else {
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node t = ((Edge*)ptr)->target;
              ptr += sizeof(Edge);
              EdgeInfo& ei =
                  m_edgeInfo[((Edge*)(m_pages[info.page] + info.posInPage))[i]
                                 .idEdge];
              unsigned* tab = (unsigned*)(m_pages[ei.page] + ei.posInPage);
              bool conflict = false;
              while (*tab) {
                d4::Lit l = d4::Lit::makeLit(*tab >> 1, *tab & 1);
                tab++;
                if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
                  conflict = true;
                  break;
                }
              }
              if (!conflict && status[t] == NOT_PROCESS) evalStack.push_back(t);
            }
          }
          break;
        }
        case IN_PROCESS: {
          status[cur] = DONE;
          uint8_t* ptr = m_pages[info.page] + info.posInPage;
          if (info.typeNode == NODE_AND) {
            subSat[cur] = 1;
            for (unsigned i = 0; i < info.nbSons; i++) {
              Node s = *((Node*)ptr);
              ptr += sizeof(Node);
              if (subSat[s] == 0) {
                subSat[cur] = 0;
                break;
              }
            }
          } else {
            subSat[cur] = 0;
            for (unsigned i = 0; i < info.nbSons; i++) {
              Edge& e = *((Edge*)ptr);
              ptr += sizeof(Edge);
              EdgeInfo& ei = m_edgeInfo[e.idEdge];
              unsigned* tab = (unsigned*)(m_pages[ei.page] + ei.posInPage);
              bool conflict = false;
              while (*tab) {
                d4::Lit l = d4::Lit::makeLit(*tab >> 1, *tab & 1);
                tab++;
                if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
                  conflict = true;
                  break;
                }
              }
              if (!conflict && subSat[e.target] == 1) {
                subSat[cur] = 1;
                break;
              }
            }
          }
          evalStack.pop_back();
          break;
        }
        case DONE:
          evalStack.pop_back();
          break;
      }
    }
    if (subSat[n] != 1) return;

    // Pass 2: backtracking enumeration with SAT pruning at OR branches.
    std::vector<Node> stack;
    stack.push_back(n);
    std::vector<d4::Lit> current_pm = assums;

    auto backtrack = [&](auto& self) -> void {
      if (stack.empty()) {
        onModel(current_pm);
        return;
      }
      Node cur = stack.back();
      stack.pop_back();
      if (cur == 1) {
        self(self);
        stack.push_back(cur);
        return;
      }

      const NodeStruct& info = m_nodeInfo[cur];
      uint8_t* ptr = m_pages[info.page] + info.posInPage;

      if (info.typeNode == NODE_AND) {
        for (int i = (int)info.nbSons - 1; i >= 0; i--)
          stack.push_back(*((Node*)(ptr + i * sizeof(Node))));
        self(self);
        for (unsigned i = 0; i < info.nbSons; i++) stack.pop_back();
        stack.push_back(cur);
      } else {
        for (unsigned i = 0; i < info.nbSons; i++) {
          Edge& e = *((Edge*)ptr);
          ptr += sizeof(Edge);
          if (subSat[e.target] != 1) continue;

          EdgeInfo& ei = m_edgeInfo[e.idEdge];
          unsigned* tab = (unsigned*)(m_pages[ei.page] + ei.posInPage);

          bool conflict = false;
          unsigned added = 0;
          unsigned* scan = tab;
          while (*scan) {
            d4::Lit l = d4::Lit::makeLit(*scan >> 1, *scan & 1);
            scan++;
            if (valuation[l.var()] != 2 && valuation[l.var()] != l.sign()) {
              conflict = true;
              break;
            }
          }
          if (conflict) continue;

          while (*tab) {
            d4::Lit l = d4::Lit::makeLit(*tab >> 1, *tab & 1);
            tab++;
            if (valuation[l.var()] == 2) {
              current_pm.push_back(l);
              added++;
            }
          }

          // SAT-based pruning: ask caller if this branch should be skipped.
          bool prune = shouldPrune(e.target, current_pm);
          if (!prune) {
            stack.push_back(e.target);
            self(self);
            stack.pop_back();
          }
          for (unsigned j = 0; j < added; j++) current_pm.pop_back();
        }
        stack.push_back(cur);
      }
    };
    backtrack(backtrack);
  }
};
}  // namespace semiring