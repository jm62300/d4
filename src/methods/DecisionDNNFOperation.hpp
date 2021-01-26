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
#pragma once
#include "DataBranch.hpp"
#include "nnf/NodeManager.hpp"

namespace d4
{
template<class U> class Operation;
template <class T, class U> class DecisionDNNFOperation : public Operation<U>
{
 private:
  ProblemManager *m_problem;
  NodeManager<T> *m_nodeManager;
  
 public:

  DecisionDNNFOperation() = delete;
  
  /**
     Constructor.

     @param[in] problem, allows to get information about the problem such as
     weights.
   */
  DecisionDNNFOperation(ProblemManager *problem, SpecManager *specs)
      : m_problem(problem)
  {
    m_nodeManager = NodeManager<T>::makeNodeManager(specs->getNbVariable() + 1);
  } // constructor.


  /**
     Create top node and returns it.

     \return a top node.
   */
  U createTop()
  {
    return NULL;
  } // createTop


  /**
     Create bottom node and returns it.

     \return a bottom node.
   */
  U createBottom()
  {
    return NULL;
  } // createBottom


  /**
     Compute the sum of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
  */
  U manageDeterministOr(DataBranch<U> *elts, unsigned size)
  {
    return NULL;
  } // manageDeterministOr

  
  /**
     Compute the product of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
   */
  U manageDecomposableAnd(U *elts, unsigned size)
  {
    return NULL;
  } // manageDecomposableAnd


  /**
     Manage the case where the problem is unsatisfiable.

     \return 0 as number of models.
   */
  U manageBottom()
  {
    return NULL;
  } // manageBottom


  /**
     Manage the case where the problem is a tautology.

     @param[in] component, the current set of variables (useless here).
     
     \return 0 as number of models.
   */
  U manageTop(std::vector<Var> &component)
  {
    return NULL;
  } // manageTop


  /**
     Manage the case where we only have a branch in our OR gate.

     @param[in] e, the branch we are considering.

     \return the number of models associate to the given branch.
   */
  U manageBranch(DataBranch<U> &e)
  {
    return NULL;
  } // manageBranch


  /**
     Manage the final result compute.

     @param[in] result, the result we are considering.
     @param[in] vm, a set of options that describes what we want to do on the
     given result.
     @param[in] out, the output stream.
   */
  void manageResult(U &result, po::variables_map &vm, std::ostream &out)
  {
    out << "s " << std::fixed << result << "\n";
  } // manageResult
};

} // d4


