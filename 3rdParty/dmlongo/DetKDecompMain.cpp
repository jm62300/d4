// det-k-decomp V1.0
//
// Reference paper: G. Gottlob and M. Samer,
// A Backtracking-Based Algorithm for Computing Hypertree-Decompositions,
// Submitted for publication.
//
// Note: This program is a prototype implementation and does in no sense
// claim to be the most efficient way of implementing det-k-decomp. Moreover,
// several parts of the code have been developed within an implementation
// framework for evaluating several decomposition algorithms. These parts of
// the code may therefore be unnecessary or are formulated in a more general
// way than would be necessary for det-k-decomp.

// det-k-decomp V2.0
//
// Reference paper: G. Gottlob and M. Samer,
// A Backtracking-Based Algorithm for Computing Hypertree-Decompositions,
// Submitted for publication.
//
// Note: This program is a prototype implementation and does in no sense
// claim to be the most efficient way of implementing det-k-decomp. Moreover,
// several parts of the code have been developed within an implementation
// framework for evaluating several decomposition algorithms. These parts of
// the code may therefore be unnecessary or are formulated in a more general
// way than would be necessary for det-k-decomp.

#define _CRT_SECURE_NO_DEPRECATE

#include <signal.h>
#include <unistd.h>

#include <cmath>
#include <ctime>
#include <iostream>

using namespace std;

#include "DetKDecomp.h"
#include "Globals.h"
#include "Hypergraph.h"
#include "Hypertree.h"
#include "PACEParser.h"

HypertreeSharedPtr decompK(HypergraphSharedPtr &, int);

volatile sig_atomic_t tle = 0;
void term(int signum) {
  std::cout << "signal received\n";
  tle = 1;
}

volatile int z = 0;
volatile bool cut = false;

int main(int argc, char **argv) {
  // Initialize random number generator
  int iRandomInit;
  unsigned int seed = 2911;  //(unsigned int)time(NULL);
  // cout << seed << endl;
  srand(seed);
  iRandomInit = random_range(999, 9999);
  for (int i = 0; i < iRandomInit; i++) rand();

  void (*prev_handler)(int);
  alarm(40);
  prev_handler = signal(SIGALRM, term);

  // Build hypergraph
  PACEParser *p = new PACEParser();
  HypergraphSharedPtr hg = p->parseInput(&cin);

  int n = hg->getNbrOfVertices();
  int m = hg->getNbrOfEdges();

  const int NUM_TRIES = 3;
  // time_t t1, t2;
  time_t inizio, fine;
  int tries = NUM_TRIES;
  cut = false;
  time(&inizio);
  int k = (int)ceil(m / 2.0);
  HypertreeSharedPtr bestHT = decompK(hg, k);
  HypertreeSharedPtr ht;
  do {
    k = bestHT->getHTreeWidth() - 1;
    cut = false;
    ht = decompK(hg, k);

    if (ht != NULL) {
      tries = NUM_TRIES;
      bestHT = ht;

      p->writeOutput(bestHT, n, m);
    } else {
      tries--;
      for (int i = 0; i < iRandomInit; i++) rand();
    }

    cout << "w= " << bestHT->getHTreeWidth() << endl;
  } while (!tle && ((ht != NULL) || (cut && (tries > 0))));

  time(&fine);
  p->writeOutput(bestHT, n, m);

  // cout << "time= " << difftime(t2, t1) << "s" << endl;
  cout << "tot= " << difftime(fine, inizio) << "s" << endl;
  cout << "w= " << bestHT->getHTreeWidth() << endl;

  delete p;
  return EXIT_SUCCESS;
}

HypertreeSharedPtr decompK(HypergraphSharedPtr &HG, int iWidth) {
  HypertreeSharedPtr HT;
  DetKDecomp Decomp(HG, iWidth, false);

  // Apply the decomposition algorithm
  HT = Decomp.buildHypertree();
  if (HT != NULL) HT->shrink(false);

  return HT;
}
