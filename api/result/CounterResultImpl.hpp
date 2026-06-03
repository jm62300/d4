#pragma once

#include <memory>
#include <sstream>
#include <limits>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include "CounterResult.hpp"

#define private public
#include "src/methods/DpllStyleMethod.hpp"
#undef private

namespace d4::api {

template <typename T, typename O>
class CounterResultImpl : public CounterResult {
 public:
  CounterResultImpl(T result, std::unique_ptr<d4::DpllStyleMethod<T, O>> method)
      : result_(result), method_(std::move(method)) {}

  std::string getResult() const override {
    std::stringstream ss;
    ss.precision(std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);
    boost::multiprecision::mpf_float::default_precision(128);
    ss << result_;
    return ss.str();
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

} // namespace d4::api
