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
#include "SpecManager.hpp"

#include "cnf/SpecManagerCnfDyn.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Generate an occurrence manager regarding the options given as parameter.

   @param[in] vm, the arguments on the command line.
   @param[in] p, a problem manager.

   \return the occurrence manager that fits the command line.
 */
SpecManager *SpecManager::makeSpecManager(po::variables_map &vm,
                                          ProblemManager &p,
                                          std::ostream &out) {
  std::string inType = vm["input-type"].as<std::string>();
  std::string meth = vm["occurrence-manager"].as<std::string>();

  out << "c [CONSTRUCTOR SPEC] Spec manager: " << meth << " " << inType << "\n";

  if (inType == "cnf" || inType == "dimacs" || inType == "tcnf") {
    if (meth == "dynamic") return new SpecManagerCnfDyn(p);
    return NULL;
  }

  if (inType == "circuit") {
    out << "c Warning: only handle the case where the circuit is translated "
           "into a CNF formula\n";
    if (meth == "dynamic") return new SpecManagerCnfDyn(p);
    return NULL;
  }

  throw(FactoryException("Cannot create a SpecManager", __FILE__, __LINE__));
}  // makeSpecManager

}  // namespace d4
