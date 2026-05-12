/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <exception>
#include <iostream>
#include <sstream>
#include <string>

namespace bipe {

/**
 * @brief Custom exception class for handling factory-related errors.
 *
 * This exception securely captures and formats the specific error message
 * along with the exact source file and line number where the failure occurred.
 * This makes debugging factory instantiation errors significantly easier.
 */
class FactoryException : public std::exception {
 private:
  std::string m_error_message;
  const char* m_file;
  int m_line;

 public:
  /**
   * @brief Constructs a new FactoryException with source location tracking.
   *
   * It is highly recommended to use the standard `__FILE__` and `__LINE__`
   * macros when invoking this constructor to automatically capture the exact
   * location.
   *
   * @param msg The descriptive error message explaining what failed.
   * @param file_ The name of the source file where the exception is thrown.
   * @param line_ The exact line number in the source file.
   */
  FactoryException(const char* msg, const char* file_, int line_)
      : m_file(file_), m_line(line_) {
    std::ostringstream o;
    o << m_file << ":" << m_line << ": " << msg;
    m_error_message = o.str();
  }

  /**
   * @brief Retrieves the formatted error description.
   *
   * The returned string is formatted as: "filename:linenumber: error_message".
   *
   * @return A pointer to a null-terminated constant character array containing
   *         the full error description.
   */
  virtual const char* what() const noexcept override {
    return m_error_message.c_str();
  }
};

}  // namespace bipe