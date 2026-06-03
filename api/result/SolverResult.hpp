#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <boost/multiprecision/gmp.hpp>
#include "c++/semirings/MpzComplexSemiring.hpp"

namespace d4::api {

/**
 * @brief Common base class for solving and compilation results, holding execution statistics.
 */
class SolverResult {
 public:
  virtual ~SolverResult() = default;

  /**
   * @brief Returns the number of recursive calls made during solving.
   */
  virtual unsigned getNbRecursiveCalls() const = 0;

  /**
   * @brief Returns the number of formula splits during solving.
   */
  virtual unsigned getNbSplits() const = 0;

  /**
   * @brief Returns the number of decision nodes created during solving.
   */
  virtual unsigned getNbDecisionNodes() const = 0;

  /**
   * @brief Returns the solve time in seconds.
   */
  virtual float getSolveTime() const = 0;
};

/**
 * @brief Represents the result of a direct model counting execution.
 */
class CountResult : public SolverResult {
 public:
  virtual ~CountResult() = default;

  /**
   * @brief Returns the model count result as a string.
   */
  virtual std::string getResult() const = 0;

  /**
   * @brief Returns the integer model count result if the formula was unweighted.
   * Throws std::runtime_error if the result type is not an integer.
   */
  virtual boost::multiprecision::mpz_int getIntResult() const {
    throw std::runtime_error("Count result is not an integer");
  }

  /**
   * @brief Returns the floating-point model count result if the formula used real weights.
   * Throws std::runtime_error if the result type is not a float.
   */
  virtual boost::multiprecision::mpf_float getFloatResult() const {
    throw std::runtime_error("Count result is not a float");
  }

  /**
   * @brief Returns the complex model count result if the formula used complex weights.
   * Throws std::runtime_error if the result type is not a complex number.
   */
  virtual semiring::Complex getComplexResult() const {
    throw std::runtime_error("Count result is not a complex number");
  }
};

/**
 * @brief Represents the result of compiling a formula into a queryable d-DNNF circuit.
 */
class CompileResult : public SolverResult {
 public:
  virtual ~CompileResult() = default;

  /**
   * @brief Returns the number of nodes in the compiled d-DNNF graph.
   */
  virtual unsigned getNbNodes() const = 0;

  /**
   * @brief Returns the number of edges in the compiled d-DNNF graph.
   */
  virtual unsigned getNbEdges() const = 0;

  /**
   * @brief Prints the compiled d-DNNF circuit in standard NNF format to a stream.
   */
  virtual void printNNF(std::ostream& out) const = 0;

  /**
   * @brief Returns the compiled d-DNNF circuit as a string in standard NNF format.
   */
  virtual std::string getNNFString() const = 0;

  /**
   * @brief Computes the model count of the compiled d-DNNF circuit under literal assumptions.
   * @param queryLits A vector of query literals represented as DIMACS integers (e.g. 1, -2).
   * @return The model count as a string.
   */
  virtual std::string count(const std::vector<int>& queryLits) const = 0;

  /**
   * @brief Checks if the formula is satisfiable under the given literal assumptions.
   * @param queryLits A vector of query literals represented as DIMACS integers.
   * @return True if SAT, false if UNSAT.
   */
  virtual bool isSAT(const std::vector<int>& queryLits) const = 0;
};

} // namespace d4::api
