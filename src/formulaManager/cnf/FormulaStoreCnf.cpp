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

#include "FormulaStoreCnf.hpp"

namespace d4 {

FormulaStoreCnf::FormulaStoreCnf(CnfManager& occM, ModeStore mdStore)
    : m_specManager(occM) {
  m_modeStore = mdStore;
  m_nbClauseCnf = occM.getNbClause();
  m_nbVarCnf = occM.getNbVariable();
  m_maxSizeClause = occM.getMaxSizeClause();
  m_varInComponent.resize(m_nbVarCnf, false);
}

void FormulaStoreCnf::collectIdActiveClauses(std::span<const Var> component,
                                             std::vector<unsigned>& idxClauses) {
  idxClauses.resize(0);
  if (m_modeStore == CACHE_ALL)
    m_specManager.getCurrentClauses(idxClauses, component);
  else
    m_specManager.getCurrentClausesNotBin(idxClauses, component);

  unsigned i, j;
  for (i = j = 0; i < idxClauses.size(); i++) {
    if (!isKeptClause(idxClauses[i])) continue;
    idxClauses[j++] = idxClauses[i];
  }
  idxClauses.resize(j);
}

}  // namespace d4
