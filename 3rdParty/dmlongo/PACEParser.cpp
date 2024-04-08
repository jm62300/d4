/*
 * PACEParser.cpp
 *
 *  Created on: 20 apr 2019
 *      Author: david
 */

#include "PACEParser.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <sstream>
#include <string>
#include <vector>

#include "Hyperedge.h"
#include "Hypergraph.h"

namespace dmlongo {

PACEParser::PACEParser() {}

PACEParser::~PACEParser() {}

std::vector<std::string> PACEParser::split(const std::string &s,
                                           char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

HypergraphSharedPtr PACEParser::parseInputFromFile(std::string filename) {
  std::ifstream file(filename);

  if (!file) {
    return nullptr;
  }

  VertexVector vertices;
  HypergraphSharedPtr h = std::make_shared<Hypergraph>();
  int n = -1;
  int m = -1;
  bool p_read = false;
  std::string line;
  while (file) {
    do {
      getline(file, line);
    } while (line.empty());
    // istringstream iss(line);
    std::vector<std::string> tokens = split(line, ' ');
    //{ istream_iterator<string> { iss }, istream_iterator<string> { } };

    std::string tk = tokens.at(0);
    if (tk != "c" && tk != "") {
      std::cout << line << std::endl;
      if (!p_read) {
        if (tk == "p") {
          n = stoi(tokens.at(2));
          m = stoi(tokens.at(3));

          for (int i = 1; i <= n; i++) {
            std::string name = std::to_string(i);
            vertices.push_back(std::make_shared<Vertex>(name));
          }
          /*
           for (int i = 0; i < n; i++)
           {
           cout << vertices.at(i)->getName() << endl;
           }
           */

          p_read = true;
        } else {
          std::cout << "error" << std::endl;
        }
      } else if (m > 0) {
        // int hedge = stoi(tk);
        HyperedgeSharedPtr edge = std::make_shared<Hyperedge>(tk);
        // cout << "edge: begin" << endl;
        for (size_t i = 1; i < tokens.size(); i++) {
          int v = stoi(tokens.at(i)) - 1;
          edge->add(vertices[v]);
        }
        h->insertEdge(edge);
        // cout << "edge: end" << endl;
        m = m - 1;
        // cout << m << endl;
        if (m == 0) break;
      } else {
        break;
      }
    }
  }
  // cout << "chiuso1" << endl;
  file.close();
  // cout << "chiuso2" << endl;

  return h;
}

HypergraphSharedPtr PACEParser::parseInput(std::istream *cin) {
  HypergraphSharedPtr hg = std::make_shared<Hypergraph>();
  VertexVector vertices;
  HyperedgeSharedPtr edge;

  int n = -1;
  int m = -1;
  bool p_read = false;
  std::string line;
  while (getline(*cin, line)) {
    if (!line.empty()) {
      std::vector<std::string> tokens = split(line, ' ');

      std::string tk = tokens.at(0);
      if (tk != "c" && tk != "") {
        // cout << line << endl;
        if (!p_read) {
          if (tk == "p") {
            n = stoi(tokens.at(2));
            m = stoi(tokens.at(3));

            for (int i = 1; i <= n; i++) {
              std::string name = std::to_string(i);
              vertices.push_back(std::make_shared<Vertex>(name));
            }
            /*
             for (int i = 0; i < n; i++)
             {
             cout << vertices.at(i)->getName() << endl;
             }
             */

            p_read = true;
          } else {
            std::cout << "error" << std::endl;
          }
        } else if (m > 0) {
          edge = std::make_shared<Hyperedge>(tk);
          // cout << "edge: begin" << endl;
          for (size_t i = 1; i < tokens.size(); i++) {
            int v = stoi(tokens.at(i)) - 1;
            edge->add(vertices[v]);
          }
          hg->insertEdge(edge);
          // cout << "edge: end" << endl;
          m = m - 1;
          // cout << m << endl;
          if (m == 0) break;
        } else {
          break;
        }
      }
    }
  }
  cin->clear();
  return hg;
}

std::string PACEParser::fromPaceToFischl(std::istream *cin) {
  std::stringstream fischl;
  int n = -1;
  int m = -1;
  bool p_read = false;
  std::string line;
  while (getline(*cin, line)) {
    if (!line.empty()) {
      std::vector<std::string> tokens = split(line, ' ');

      std::string tk = tokens.at(0);
      if (tk != "c" && tk != "") {
        std::cout << line << std::endl;
        if (!p_read) {
          if (tk == "p") {
            n = stoi(tokens.at(2));
            m = stoi(tokens.at(3));
            p_read = true;
          } else {
            std::cout << "error" << std::endl;
          }
        } else if (m > 0) {
          fischl << tk << "(";
          for (size_t i = 1; i < tokens.size(); i++) {
            fischl << tokens.at(i);
            if (i < tokens.size() - 1) {
              fischl << ",";
            }
          }
          fischl << ")";
          m = m - 1;
          if (m == 0) {
            fischl << "." << std::endl;
            break;
          } else {
            fischl << "," << std::endl;
          }
        } else {
          break;
        }
      }
    }
  }
  cin->clear();
  return fischl.str();
}

void PACEParser::writeOutput(HypertreeSharedPtr ht, int n, int m) {
  int numBags = 0;
  int width = ht->getHTreeWidth();
  std::stringstream bagsD, treeD, widthD;

  std::list<HypertreeSharedPtr> *queue = new std::list<HypertreeSharedPtr>();
  queue->push_back(ht);
  int id = 0;
  int nextID = 2;
  while (!queue->empty()) {
    HypertreeSharedPtr curr = queue->front();
    queue->pop_front();
    numBags++;
    id++;
    bagsD << "b " << id;
    for (auto v : curr->getChi()) {
      bagsD << " " << v->getName();
    }
    bagsD << '\n';

    for (auto e : curr->getLambda()) {
      widthD << "w " << id << " " << e->getName() << " 1\n";
    }
    for (auto c : curr->allChildren()) {
      treeD << id << " " << nextID << '\n';
      nextID++;
      queue->push_back(c);
    }
  }
  delete queue;

  std::cout << "s htd " << numBags << " " << width << " " << n << " " << m
            << std::endl;
  // cout << bagsD.str() << treeD.str() << widthD.str() << endl;
}

}  // namespace dmlongo