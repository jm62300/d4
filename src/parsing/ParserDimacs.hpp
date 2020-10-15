#ifndef d4_parsing_ParserDimacs_hpp
#define d4_parsing_ParserDimacs_hpp

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>

#include "../utils/BufferRead.hpp"


namespace d4
{
class ParserDimacs
{
 private:
  int parse_DIMACS_main(BufferRead &in, std::vector< std::vector<int> > &clauses);

 public:
  int parse_DIMACS(std::string input_stream, std::vector< std::vector<int> > &clauses);
};
} // d4

#endif
