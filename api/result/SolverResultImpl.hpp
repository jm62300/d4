#pragma once

#include <memory>
#include <sstream>
#include <limits>
#include <vector>
#include <map>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <boost/multiprecision/gmp.hpp>

#include "SolverResult.hpp"

#define private public
#include "src/methods/DpllStyleMethod.hpp"
#undef private

#include "c++/semirings/DecDNNFSemiring.hpp"

namespace d4::api {

namespace mpz = boost::multiprecision;

template <typename T, typename O>
class CountResultImpl : public CountResult {
 public:
  CountResultImpl(T result, std::unique_ptr<d4::DpllStyleMethod<T, O>> method)
      : result_(result), method_(std::move(method)) {}

  std::string getResult() const override {
    std::stringstream ss;
    ss.precision(std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);
    boost::multiprecision::mpf_float::default_precision(128);
    ss << result_;
    return ss.str();
  }

  boost::multiprecision::mpz_int getIntResult() const override {
    if constexpr (std::is_same_v<T, boost::multiprecision::mpz_int>) {
      return result_;
    } else {
      throw std::runtime_error("Count result is not an integer");
    }
  }

  boost::multiprecision::mpf_float getFloatResult() const override {
    if constexpr (std::is_same_v<T, boost::multiprecision::mpf_float>) {
      return result_;
    } else {
      throw std::runtime_error("Count result is not a float");
    }
  }

  semiring::Complex getComplexResult() const override {
    if constexpr (std::is_same_v<T, semiring::Complex>) {
      return result_;
    } else {
      throw std::runtime_error("Count result is not a complex number");
    }
  }

  unsigned getNbRecursiveCalls() const override {
    return method_ ? method_->m_nbCallCall : 0;
  }

  unsigned getNbSplits() const override {
    return method_ ? method_->m_nbSplit : 0;
  }

  unsigned getNbDecisionNodes() const override {
    return method_ ? method_->m_nbDecisionNode : 0;
  }

  float getSolveTime() const override {
    return method_ ? method_->getTimer() : 0.0f;
  }

 private:
  T result_;
  std::unique_ptr<d4::DpllStyleMethod<T, O>> method_;
};

class CompileResultImpl : public CompileResult {
 public:
  CompileResultImpl(semiring::Node resultNode,
                    std::unique_ptr<d4::DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>> method,
                    unsigned nbVar,
                    const std::map<d4::Lit, std::string>& weightMap)
      : resultNode_(resultNode),
        method_(std::move(method)),
        nbVar_(nbVar),
        isWeighted_(!weightMap.empty()) {
    
    // Construct the weights vectors
    noweight_.resize(2 + nbVar_ * 2, mpz::mpz_int(1));
    weight_.resize(2 + nbVar_ * 2, mpz::mpf_float(1));
    for (const auto& [lit, w] : weightMap) {
      weight_[lit.intern()] = mpz::mpf_float(w);
    }
  }

  unsigned getNbNodes() const override {
    if (!method_) return 0;
    return method_->getSemiring().getNbNodes();
  }

  unsigned getNbEdges() const override {
    if (!method_) return 0;
    return method_->getSemiring().getNbEdges();
  }

  void printNNF(std::ostream& out) const override {
    if (!method_) return;
    method_->getSemiring().printNNF(resultNode_, out);
  }

  std::string getNNFString() const override {
    std::stringstream ss;
    printNNF(ss);
    return ss.str();
  }

  std::string count(const std::vector<int>& queryLits) const override {
    if (!method_) return "0";
    std::vector<d4::Lit> assums = translateQuery(queryLits);
    const semiring::DecDNNFSemiring& semiring = method_->getSemiring();

    std::stringstream ss;
    ss.precision(std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);
    boost::multiprecision::mpf_float::default_precision(128);

    if (isWeighted_) {
      ss << semiring.count<mpz::mpf_float>(resultNode_, assums, weight_, nbVar_);
    } else {
      ss << semiring.count<mpz::mpz_int>(resultNode_, assums, noweight_, nbVar_);
    }
    return ss.str();
  }

  bool isSAT(const std::vector<int>& queryLits) const override {
    if (!method_) return false;
    std::vector<d4::Lit> assums = translateQuery(queryLits);
    const semiring::DecDNNFSemiring& semiring = method_->getSemiring();
    return semiring.isSAT(resultNode_, assums, nbVar_);
  }

  unsigned getNbRecursiveCalls() const override {
    return method_ ? method_->m_nbCallCall : 0;
  }

  unsigned getNbSplits() const override {
    return method_ ? method_->m_nbSplit : 0;
  }

  unsigned getNbDecisionNodes() const override {
    return method_ ? method_->m_nbDecisionNode : 0;
  }

  float getSolveTime() const override {
    return method_ ? method_->getTimer() : 0.0f;
  }

 private:
  std::vector<d4::Lit> translateQuery(const std::vector<int>& queryLits) const {
    std::vector<d4::Lit> d4Query;
    d4Query.reserve(queryLits.size());
    for (int lit : queryLits) {
      if (lit != 0) {
        d4Query.push_back((lit > 0) ? d4::Lit::makeLit(lit, false)
                                    : d4::Lit::makeLit(-lit, true));
      }
    }
    return d4Query;
  }

  semiring::Node resultNode_;
  std::unique_ptr<d4::DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>> method_;
  unsigned nbVar_;
  bool isWeighted_;
  std::vector<mpz::mpz_int> noweight_;
  std::vector<mpz::mpf_float> weight_;
};

} // namespace d4::api
