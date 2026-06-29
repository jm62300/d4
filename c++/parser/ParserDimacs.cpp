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
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "ParserDimacs.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

// #include "src/utils/Parsing.hpp"

namespace parser {

/**
 * @brief Parse the dimacs format in order to extract CNF formula and
 * different set of variables such projected variables, max variables, ...
 * but also variables' weights if it is the case that the problem is
 * weighted.
 *
 * @param in, the stream buffer where are read the information.
 * @param problemManager, the place where is store the result.
 * @return an integer that gives the problem's number of variables.
 */
int ParserDimacs::parse_DIMACS_main(BufferRead& in, Formula& formula) {
  std::vector<int> lits;
  std::string s;

  std::vector<std::vector<int>>& clauses = formula.clauses;

  int nbVars = 0;
  int nbClauses = 0;

  int cpt = 0;
  char previousChar = '\0';
  formula.projected = false;
  formula.weightType = WeightType::INT;
  formula.type = "cnf";

  std::vector<int> showedVars, maxVars, indVars;

  for (;;) {
    in.skipSpace();
    if (in.eof()) break;

    if (in.currentChar() == 'p') {
      in.consumeChar();
      in.skipSpace();

      bool vpActivated = false;
      if (in.currentChar() == 'p') {
        vpActivated = true;
        in.consumeChar();
      }
      if (in.currentChar() == 'w') in.consumeChar();

      if (in.nextChar() != 'c' || in.nextChar() != 'n' ||
          in.nextChar() != 'f') {
        throw std::runtime_error("PARSE ERROR! Unexpected char after p");
      }

      nbVars = in.nextInt();
      nbClauses = in.nextInt();

      if (vpActivated)
        std::cout << "c Some variable are marked: " << in.nextInt() << "\n";

      if (nbClauses < 0) {
        throw std::runtime_error("PARSE ERROR! Negative number of clauses");
      }
    } else if (in.currentChar() == 'z') {
      std::cout << "c [PARSER] Read EOF in the file\n";
      break;
    } else if (in.currentChar() == 'c') {
      in.consumeChar();
      in.skipSimpleSpace();

      if (in.currentChar() == 't') {
        in.consumeChar();
        in.skipSimpleSpace();
        formula.typeCompet = in.lineWord();
        in.skipLine();
      } else if (in.canConsume("p")) {
        if (in.canConsume("weight")) {
          int lit = in.nextInt();
          std::vector<std::string> elements;
          std::istringstream iss(in.lineWord());
          std::string current_word;

          while (iss >> current_word) {
            elements.push_back(current_word);
          }

          // test if it is correct.
          if (elements.empty() || elements.back() != "0") {
            throw std::runtime_error(
                "PARSE ERROR! Weight line must be terminated by 0");
          }

          // remove the zero at the end.
          elements.pop_back();

          // Complex weight written as a single token "<real>±<imag>i",
          // e.g. "0+1i", "0.70710678...+0i" or "0-0.70710678...i". Split it
          // into its real and imaginary parts.
          if (elements.size() == 1 && !elements[0].empty() &&
              elements[0].back() == 'i') {
            std::string tok = elements[0];
            tok.pop_back();  // drop the trailing 'i'

            // The imaginary part starts at the last '+'/'-' that is neither
            // the leading sign of the number nor part of an exponent (e.g.
            // the '-' in "1.5e-10").
            std::size_t split = std::string::npos;
            for (std::size_t i = tok.size(); i-- > 1;) {
              if ((tok[i] == '+' || tok[i] == '-') && tok[i - 1] != 'e' &&
                  tok[i - 1] != 'E') {
                split = i;
                break;
              }
            }
            if (split == std::string::npos) {
              throw std::runtime_error(
                  "PARSE ERROR! Malformed complex weight: " + elements[0]);
            }

            std::string im = tok.substr(split);  // keeps its sign
            if (im[0] == '+') im.erase(0, 1);

            elements[0] = tok.substr(0, split);
            elements.push_back(im);
          }

          if (elements.size() == 1) {
            formula.weightMap[lit] = elements[0];
            if (formula.weightType == WeightType::INT)
              formula.weightType = WeightType::FLOAT;
          } else {
            formula.weightMap[lit] = elements[0] + " " + elements[1];
            formula.weightType = WeightType::COMPLEX;
          }
        } else if (in.canConsume("complex")) {
          int lit = in.nextInt();
          std::string wr = in.nextWord();
          std::string wi = in.nextWord();
          formula.weightMap[lit] = wr + " " + wi;
          formula.weightType = WeightType::COMPLEX;

          // in this format we have an end line we have to consume.
          [[maybe_unused]] int endLine = in.nextInt();
          if (endLine != 0) {
            throw std::runtime_error(
                "PARSE ERROR! Complex weight line must be terminated by 0");
          }
        } else if (in.canConsume("show"))
          in.readListIntTerminatedByZero(showedVars);
        else
          in.skipLine();
      } else if (in.canConsume("max")) {
        in.readListIntTerminatedByZero(maxVars);
      } else if (in.canConsume("ind")) {
        in.readListIntTerminatedByZero(indVars);
      } else {
        in.skipLine();
      }
    } else {
      lits.clear();
      int v = -1;
      do {
        v = in.nextInt();
        if ((v > 0 && nbVars < v) || (-v > 0 && nbVars < -v)) {
          throw std::runtime_error(
              "PARSE ERROR! Number of variables incorrect: " +
              std::to_string(v));
        }

        if (v) lits.push_back(v);
      } while (v);

      if (lits.empty()) {
        throw std::runtime_error("PARSE ERROR! Clause is empty");
      }
      std::sort(lits.begin(), lits.end());

      // remove redundant literal and check for tautology.
      unsigned j = 1;
      bool isSat = false;
      for (unsigned i = 1; !isSat && i < lits.size(); i++) {
        if (lits[i] == lits[j - 1]) continue;
        isSat = lits[i] == -lits[j - 1];
        lits[j++] = lits[i];
      }

      // add the clause only if not SAT.
      if (!isSat) {
        lits.resize(j);
        clauses.push_back(lits);
      }
    }
  }

  if (maxVars.size() || indVars.size()) {
    formula.quantifications.push_back(maxVars);
    formula.quantifications.push_back(indVars);
  }
  formula.quantifications.push_back(showedVars);
  formula.nbVar = nbVars;
  formula.projected = showedVars.size() > 0;
  return nbVars;
}

int ParserDimacs::parse_DIMACS(const std::string& input_stream,
                               Formula& formula) {
  BufferRead in(input_stream);
  return parse_DIMACS_main(in, formula);
}  // parse_DIMACS

int ParserDimacs::parse_DIMACS(const char* data, size_t len, Formula& formula) {
  BufferRead in(data, len);
  return parse_DIMACS_main(in, formula);
}  // parse_DIMACS

int ParserDimacs::parse_DIMACS_from_data(const std::string& data,
                                         Formula& formula) {
  return parse_DIMACS(data.data(), data.size(), formula);
}  // parse_DIMACS_from_data

}  // namespace parser
