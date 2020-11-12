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

#include "DataInfoCnf.hpp"

namespace d4
{

/**
   Constructor.
 */
DataInfoCnf::DataInfoCnf(unsigned szData,         
                         unsigned nbVar,          
                         unsigned nbLit,          
                         unsigned nbClause,       
                         unsigned nbOctetsData,   
                         unsigned nbOctetsVar,
                         unsigned count) :
    DataInfo(szData, nbVar, nbOctetsData, nbOctetsVar, count)
{
  info1 = info1 |
          ((uint64_t) nbLit << 21) |
          ((uint64_t) nbClause << 42);
} // constructor


void DataInfoCnf::print(char *data, std::ostream &out)
{
  out << "Bucket size = " << szData() << "\n" 
      << "nbVar = " << nbVar() << "\n"
      << "nbClause = " << nbClause() << "\n"
      << "nbLit = " << nbLit() << "\n"
      << "count = " << count() << "\n"
      << "dirty = "<< dirty() << "\n";

  // print the variable
  out << "Var: " << nbVar() << "(" << nbOctetsVar() << ")\n";
  switch(nbOctetsVar())
  {
    case 1 : printData<char>(data, nbVar()); break;
    case 2 : printData<char16_t>(data, nbVar()); break;
    default : printData<char32_t>(data, nbVar()); break;
  }

  // distribution
  char *dataClause = &data[nbVar() * nbOctetsVar()];

  out << "Clause: " << nbClause() << "(" << nbOctetsData() << ")\n";
  switch(nbOctetsData())
  {
    case 1 : printData<char>(dataClause, &data[szData()] - dataClause); break;
    case 2 : printData<char16_t>(dataClause, &data[szData()] - dataClause); break;
    default : printData<char32_t>(dataClause, &data[szData()] - dataClause); break;
  }

  out << "All data: \n";
  for(unsigned i = 0 ; i<szData() ; i++) out << std::hex << data[i] << " ";
  out << "\n";
  out << "------------------------------------------\n";
}
}
