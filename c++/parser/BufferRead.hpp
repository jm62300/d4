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
#pragma once

#include <fcntl.h>

#include <boost/multiprecision/gmp.hpp>
#include <iostream>
#include <string>

#define BUFFER_SIZE 65536

#ifdef _MSC_VER
// Tell MSVC to allow standard POSIX names (open, read, close) without warnings
// or macros
#ifndef _CRT_NONSTDC_NO_WARNINGS
#define _CRT_NONSTDC_NO_WARNINGS
#endif
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace parser {

class BufferRead {
  int pos;
  int size;
  char buffer[BUFFER_SIZE];
  int m_fd;
  int m_keepOpen;
  const char* m_mem_data = nullptr;
  size_t m_mem_len = 0;
  size_t m_mem_pos = 0;
  bool m_is_mem = false;

 public:
  BufferRead(const std::string& name, bool keepOpen = false) {
    pos = 0;
    size = 0;
    m_keepOpen = keepOpen;
    m_mem_data = nullptr;
    m_mem_len = 0;
    m_mem_pos = 0;
    m_is_mem = false;

    m_fd = open(name.c_str(), O_RDONLY);
    if (m_fd < 0)
      std::cerr << "ERROR! Could not open file: " << name << "\n", exit(1);

    // fill the buffer
    size = read(m_fd, buffer, BUFFER_SIZE);
    if (size < 0) {
      perror("read()");
      exit(EXIT_FAILURE);
    }
  }

  BufferRead(const int fd, bool keepOpen = false) {
    pos = 0;
    size = 0;
    m_fd = fd;
    m_keepOpen = keepOpen;
    m_mem_data = nullptr;
    m_mem_len = 0;
    m_mem_pos = 0;
    m_is_mem = false;

    // fill the buffer
    size = read(m_fd, buffer, BUFFER_SIZE);
    if (size < 0) {
      perror("read()");
      exit(EXIT_FAILURE);
    }
  }

  BufferRead(const char* data, size_t len) {
    pos = 0;
    m_fd = 0;
    m_keepOpen = false;
    m_mem_data = data;
    m_mem_len = len;
    m_is_mem = true;

    size = std::min(len, (size_t)BUFFER_SIZE);
    if (size > 0) {
      std::copy(data, data + size, buffer);
    }
    m_mem_pos = size;
  }

  ~BufferRead() {
    if (m_fd && !m_keepOpen) close(m_fd);
  }

  inline char currentChar() { return buffer[pos]; }
  inline char nextChar() {
    char c = buffer[pos];
    consumeChar();
    return c;
  }

  inline void consumeChar() {
    pos++;
    if (pos >= size) {
      pos = 0;
      if (m_is_mem) {
        size = std::min(m_mem_len - m_mem_pos, (size_t)BUFFER_SIZE);
        if (size > 0) {
          std::copy(m_mem_data + m_mem_pos, m_mem_data + m_mem_pos + size, buffer);
          m_mem_pos += size;
        }
      } else {
        size = read(m_fd, buffer, BUFFER_SIZE);
        if (size < 0) {
          perror("read()");
          exit(EXIT_FAILURE);
        }
      }
    }
  }

  inline bool eof() { return !size; }
  inline void skipSpace() {
    while (!eof() && (currentChar() == ' ' || currentChar() == '\t' ||
                      currentChar() == '\n' || currentChar() == '\r'))
      consumeChar();
  }

  inline void skipSimpleSpace() {
    while (!eof() && (currentChar() == ' ' || currentChar() == '\t'))
      consumeChar();
  }

  inline void skipLine() {
    while (!eof() && currentChar() != '\n') consumeChar();
    consumeChar();
  }

  inline int nextInt() {
    int ret = 0;
    skipSpace();

    bool sign = currentChar() == '-';
    if (sign) consumeChar();
    while (!eof() && currentChar() >= '0' && currentChar() <= '9') {
      ret = ret * 10 + (nextChar() - '0');
    }
    return (sign) ? -ret : ret;
  }

  /**
   * @brief Check out if the given modif can be consumed.
   *
   * @param motif, the string we want to consume.
   * @return true if the modif can be consumed, false otherwise.
   */
  inline bool canConsume(std::string motif) {
    skipSimpleSpace();
    for (auto c : motif) {
      if (currentChar() != c)
        return false;
      else
        consumeChar();
    }
    return true;
  }  // canConsume

  /**
   * @brief Read the next integer in the given stream while the value 0 is not
   * reached.
   *
   * @param in, the stream.
   * @param list, the list of integer we parsed.
   */
  void readListIntTerminatedByZero(std::vector<int>& list) {
    int v = -1;
    do {
      v = nextInt();
      if (v) list.push_back(v);
    } while (v);
  }  // readListIntTerminatedByZero

  inline double nextDouble() {
    skipSpace();

    bool sign = currentChar() == '-';
    if (sign) consumeChar();

    std::string cur = "";
    while (!eof() && ((currentChar() >= '0' && currentChar() <= '9') ||
                      currentChar() == '.' || currentChar() == 'e' ||
                      currentChar() == '-')) {
      cur += currentChar();
      nextChar();
    }

    std::string::size_type pos = 0;
    double ret = 0;
    ret = std::stod(cur, &pos);

    return (sign) ? -ret : ret;
  }

  /**
   * @brief Read on the buffer the next word.
   *
   * @return an std::string we read.
   */
  inline std::string nextWord() {
    skipSpace();
    bool sign = currentChar() == '-';
    if (sign) consumeChar();

    std::string cur = "";
    while (!eof() && (currentChar() != ' ' && currentChar() != '\n' &&
                      currentChar() != '\t' && currentChar() != '\r')) {
      cur += currentChar();
      nextChar();
    }

    return cur;
  }

  /**
   * @brief Read on the buffer until the end of the line.
   *
   * @return an std::string we read.
   */
  inline std::string lineWord() {
    skipSpace();
    bool sign = currentChar() == '-';
    if (sign) consumeChar();

    std::string cur = "";
    while (!eof() && currentChar() != '\n') {
      cur += currentChar();
      nextChar();
    }

    return cur;
  }
};
}  // namespace parser
