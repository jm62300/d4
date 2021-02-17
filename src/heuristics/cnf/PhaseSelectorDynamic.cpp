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
#include "PhaseSelectorDynamic.hpp"


namespace d4
{

/**
   Constructor.

   @param[in] limitPhase, give the limit number of variables before switching.
*/
PhaseSelectorDynamic::PhaseSelectorDynamic(
    PartitioningHeuristicStatic *staticPartitioner) :
    PhaseSelectorManager(staticPartitioner)
{
  std::cout << "c [CONSTRUCTOR] Switching between static and dynamic decomposition:"
            << " dynamic\n";  
} // constructor


/**
   Check out if the current tree decomposition is still OK. To do it we check
   out if the current decomposition is not too unbalanced.

   @param[in] component, the current set of variables.
 */
bool PhaseSelectorDynamic::isStillOk(std::vector<Var> &component)
{
  if(!component.size() || component.size() <= 10) return true;
  
  std::vector<unsigned> &bucketNumber = m_staticPartitioner->getBucketNumber();
  std::vector<unsigned> &separatorLevel = m_staticPartitioner->getSeparatorLevel();

  unsigned cptCut = 1, left = 0, right = 0;
  
  unsigned minLevel = bucketNumber[component[0]];
  unsigned limit = separatorLevel[minLevel];

  for(unsigned i = 1 ; i<component.size() ; i++)
  {
    unsigned tmpLevel = bucketNumber[component[i]];
    if(tmpLevel == minLevel) cptCut++;
    else
    {
      if(tmpLevel < minLevel)
      {
        // set the new limit 
        limit = separatorLevel[tmpLevel];

        // select if the previous sub-tree go to the left or the right        
        if(minLevel >= limit)
        {
          right = left + right + cptCut;
          left = 0;
        }
        else
        {
          left = left + right + cptCut;
          right = 0;
        }

        cptCut = 1;
        minLevel = tmpLevel;
      }
      else // > minLevel
      {
        if(tmpLevel >= limit) right++;
        else left++;
      }
    }
  }
  
  if(left < cptCut || right < cptCut) return false;
  return true;
} // isStillok

} // d4
