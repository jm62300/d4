/*
 * PACEParser.h
 *
 *  Created on: 20 apr 2019
 *      Author: david
 */

#ifndef PACEPARSER_H_
#define PACEPARSER_H_

#include <string>

#include "Hypergraph.h"
#include "Hypertree.h"

namespace dmlongo {

class PACEParser {
 public:
  PACEParser();
  ~PACEParser();

  HypergraphSharedPtr parseInputFromFile(std::string filename);
  HypergraphSharedPtr parseInput(std::istream *cin);
  std::string fromPaceToFischl(std::istream *cin);
  void writeOutput(HypertreeSharedPtr ht, int n, int m);

 private:
  std::vector<std::string> split(const std::string &s, char delimiter);
  // string getVariable(int i) const;
  // string getAtom(int i) const;
  // int getNbrOfVars(int atom) const;
  // int getNextAtomVar();
};

}  // namespace dmlongo

#endif /* PACEPARSER_H_ */
