#include <unistd.h>

#include <iostream>
#include <vector>

#include "src/pace.h"

using namespace flowCutter;

int main(int argc, char **argv) {
  std::vector<std::pair<unsigned, unsigned>> graph;
  graph.push_back(std::make_pair(1, 2));
  graph.push_back(std::make_pair(2, 3));
  graph.push_back(std::make_pair(3, 4));
  graph.push_back(std::make_pair(4, 5));

  const char *decomp = paceMain(5, graph);
  std::cout << "print the decomposition:\n" << decomp;

  return 0;
}