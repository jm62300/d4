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

#include "DataInfoCnfCl.hpp"

namespace d4
{

/**
   Constructor.
 */
DataInfoCnfCl::DataInfoCnfCl(unsigned szData,
                           unsigned nbVar,
                           unsigned nbLit,
                           unsigned nbDiffSizeDistrib,
                           unsigned nbOctetsVar,
                           unsigned nbOctetsLit,
                           unsigned nbOctetsDistrib,
                           unsigned count) :
    DataInfo(szData, nbVar, nbOctetsLit, nbOctetsVar, count)
{
  info1 = info1 |
          (((uint64_t) nbLit) << 21) |
          (((uint64_t) nbDiffSizeDistrib) << 42);

  info2 = info2 |
          ((uint32_t) (nbOctetsDistrib - 1));
} // constructor


void DataInfoCnfCl::print(char *data, std::ostream &out)
{
  out << "Bucket size = " << szData() << "\n" 
      << "nbVar = " << nbVar() << "\n"
      << "nbDistrib = " << nbDistrib() << "\n"
      << "nbLit = " << nbLit() << "\n"
      << "count = " << count() << "\n"
      << "dirty = "<< dirty() << "\n";

  // print the variable
  out << "Var: " << nbVar() << "(" << nbOctetsVar() << ")\n";
  switch(nbOctetsVar())
  {
    case 1 : printData<uint8_t>(data, nbVar(), out); break;
    case 2 : printData<uint16_t>(data, nbVar(), out); break;
    default : printData<uint32_t>(data, nbVar(), out); break;
  }
  
  // distribution
  char *dataDistrib = &data[nbVar() * nbOctetsVar()];

  out << "Distribution: " << nbDistrib() << "(" << nbOctetDistrib() << ")\n";
  switch(nbOctetDistrib())
  {
    case 1 : printData<uint8_t>(dataDistrib, nbDistrib()<<1, out); break;
    case 2 : printData<uint16_t>(dataDistrib, nbDistrib()<<1, out); break;
    default : printData<uint32_t>(dataDistrib, nbDistrib()<<1, out); break;
  }

  out << "Clauses: " << nbLit() << "("<< nbOctetLit() << ")\n";
  char *dataClause = &dataDistrib[(nbDistrib()<<1) * nbOctetDistrib()];
  switch(nbOctetLit())
  {
    case 1 : printData<uint8_t>(dataClause, nbLit(), out); break;
    case 2 : printData<uint16_t>(dataClause, nbLit(), out); break;
    default : printData<uint32_t>(dataClause, nbLit(), out); break;
  }
  
  out << "All data: " << szData() << "\n";
  // for(unsigned i = 0 ; i<szData() ; i++) out << std::hex << (uint8_t) data[i] << " ";
  // out << "\n";
  out << "------------------------------------------\n";
} // print
}
