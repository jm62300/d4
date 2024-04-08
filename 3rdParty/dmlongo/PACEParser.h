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

class PACEParser
{
public:
	PACEParser();
	~PACEParser();

	HypergraphSharedPtr parseInputFromFile(string filename);
	HypergraphSharedPtr parseInput(istream *cin);
	string fromPaceToFischl(istream *cin);
	void writeOutput(HypertreeSharedPtr ht, int n, int m);

private:
	vector<string> split(const string& s, char delimiter);
	//string getVariable(int i) const;
	//string getAtom(int i) const;
	//int getNbrOfVars(int atom) const;
	//int getNextAtomVar();
};


#endif /* PACEPARSER_H_ */
