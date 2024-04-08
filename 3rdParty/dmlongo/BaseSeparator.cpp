#include "BaseSeparator.h"

namespace dmlongo {

BaseSeparator::BaseSeparator() {}

BaseSeparator::~BaseSeparator() {}

bool operator==(const std::shared_ptr<BaseSeparator>& lhs,
                const std::shared_ptr<BaseSeparator>& rhs) {
  return lhs->covers() == rhs->covers();
}
}  // namespace dmlongo