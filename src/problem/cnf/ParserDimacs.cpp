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
