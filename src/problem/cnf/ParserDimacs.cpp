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

#include "ParserDimacs.hpp"

namespace d4
{
int ParserDimacs::parse_DIMACS_main(BufferRead &in, std::vector< std::vector<Lit> > &clauses)
{
  std::vector<Lit> lits;
  std::string s;

  int nbVars = 0;
  int nbClauses = 0;

  for (;;)
  {
    in.skipSpace();
    if (in.eof()) break;

    if (in.currentChar() == 'p')
    {
      in.consumeChar();
      in.skipSpace();
      if(in.nextChar() != 'c' || in.nextChar() != 'n' || in.nextChar() != 'f')
        std::cerr << "PARSE ERROR! Unexpected char: " << in.currentChar() << "\n", exit(3);

      nbVars = in.nextInt();
      nbClauses = in.nextInt();
      if (nbClauses < 0) printf("parse error\n"), exit(2);
    }
    else if (in.currentChar() == 'c') in.skipLine();
    else
    {
      lits.clear();
      int v = -1;
      do
      {
        v = in.nextInt();
        if ((v > 0 && nbVars < v) || (-v > 0 && nbVars < -v))
          std::cerr << "PARSE ERROR! Number of variables given incorrect: " << v << "\n", exit(3);
        
        if(v) lits.push_back((v > 0) ? Lit(v) : Lit(-v, true));
      } while(v);

      assert(lits.size());
      clauses.push_back(lits);
    }
  }

  return nbVars;
}



int ParserDimacs::parse_DIMACS(std::string input_stream, std::vector< std::vector<Lit> > &clauses)
{
  BufferRead in(input_stream);
  return parse_DIMACS_main(in, clauses);
}// parse_DIMACS
}
