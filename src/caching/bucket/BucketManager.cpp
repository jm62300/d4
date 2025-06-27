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

#include "BucketManager.hpp"

#include "cnf/BucketManagerCnfCl.hpp"
#include "cnf/BucketManagerCnfCombi.hpp"
#include "cnf/BucketManagerCnfIndex.hpp"
#include "cnf/BucketManagerCnfSym.hpp"

namespace d4 {

/**
 * @brief BucketManager::getBucketMangerCnf implementation.
 */
BucketManager *BucketManager::getBucketMangerCnf(CnfManager &scnf,
                                                 OptionBucketManager options) {
  switch (options.clauseRepresentation) {
    case CACHE_CLAUSE:
      return new BucketManagerCnfCl(scnf, options.modeStore,
                                    options.sizeFirstPage,
                                    options.sizeAdditionalPage);
    case CACHE_SYM:
      return new BucketManagerCnfSym(scnf, options.modeStore,
                                     options.sizeFirstPage,
                                     options.sizeAdditionalPage);
    case CACHE_INDEX:
      return new BucketManagerCnfIndex(scnf, options.modeStore,
                                       options.sizeFirstPage,
                                       options.sizeAdditionalPage);
    case CACHE_COMBI:
      return new BucketManagerCnfCombi(
          scnf, options.modeStore, options.sizeFirstPage,
          options.sizeAdditionalPage, options.limitNbVarSym,
          options.limitNbVarIndex);
  }

  return NULL;
}  // getBucketManagerCnf

/**
 * @brief BucketManager::makeBucketManager implementation.
 */
BucketManager *BucketManager::makeBucketManager(OptionBucketManager options,
                                                FormulaManager &s,
                                                std::ostream &out) {
  out << "c [BUCKET MANAGER] " << options << "\n";

  switch (s.getProblemInputType()) {
    case PB_CIRC:
      try {
        CircuitWithCnfManager &ps = dynamic_cast<CircuitWithCnfManager &>(s);
        return BucketManager::getBucketMangerCnf(*(ps.getCnfManager()),
                                                 options);
      } catch (std::bad_cast &bc) {
        std::cerr << "c bad_cast caught: " << bc.what() << '\n';
        std::cerr << "c A Circuit CNF manager was expeted\n";
        exit(ERROR_BAD_CAST);
      }
    case PB_QBF:
    case PB_TCNF:
    case PB_CNF:
      try {
        CnfManager &scnf = dynamic_cast<CnfManager &>(s);
        return BucketManager::getBucketMangerCnf(scnf, options);
      } catch (std::bad_cast &bc) {
        std::cerr << "c bad_cast caught: " << bc.what() << '\n';
        std::cerr << "c A CNF formula was expeted\n";
        exit(ERROR_BAD_CAST);
      }
    case PB_NONE:
      std::cerr << "c The problem type cannot be none!\n";
      exit(ERROR_BAD_TYPE_PROBLEM);
  }

  throw(FactoryException("Cannot create a BucketManager", __FILE__, __LINE__));
}  // makeBucketManager

}  // namespace d4