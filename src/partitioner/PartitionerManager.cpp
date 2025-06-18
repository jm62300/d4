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
#include "PartitionerManager.hpp"

#include "PartitionerPatoh.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief PartitionerManager::makePartitioner implementation.
 */
PartitionerManager *PartitionerManager::makePartitioner(
    PartitionerName partitioner, const InfoHyperGraph &infoHyperGraph,
    std::ostream &out) {
  switch (partitioner) {
    case PARTITIONER_PATOH:
      return new PartitionerPatoh(infoHyperGraph, out);
  }

  throw(FactoryException("Partitioner name unknown", __FILE__, __LINE__));
}  // makePartitioner
}  // namespace d4
