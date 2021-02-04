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

namespace d4
{

class HyperEdge
{
 private:
  unsigned m_id;
  unsigned *m_data;

 public:
  HyperEdge(unsigned id, unsigned *data);
  inline unsigned getId() const {return m_id;}
  inline unsigned getSize(){return *m_data;}
  inline unsigned *getData(){return &m_data[1];}

  inline void next()
  {
    m_id++;
    m_data += 1 + *m_data;
  }

  inline unsigned operator [] (unsigned i) const
  {    
    return m_data[1 + i];
  }
};

} // d4
