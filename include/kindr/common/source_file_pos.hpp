/*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary fokindrs, with or without
 * modification, are pekindritted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary fokindr must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written pekindrission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#ifndef KINDR_COMMON_SOURCE_FILE_POS_HPP
#define KINDR_COMMON_SOURCE_FILE_POS_HPP

#include <string>
#include <iostream>
#include <sstream>
// A class and macro that gives you the current file position.

namespace kindr {
namespace internal {

class source_file_pos
{
 public:
  std::string function;
  std::string file;
  int line;

  source_file_pos(std::string function, std::string file, int line) :
    function(function), file(file), line(line) {}

  operator std::string()
  {
    return toString();
  }

  std::string toString() const
  {
    std::stringstream s;
    s << file << ":" << line << ": " << function << "()";
    return s.str();
  }

};

} // namespace internal
} // namespace kindr

inline std::ostream & operator<<(std::ostream & out, const kindr::internal::source_file_pos & sfp)
{
  out << sfp.file << ":" << std::to_string(sfp.line) << ": " << sfp.function << "()";
  return out;
}


#define KINDR_SOURCE_FILE_POS kindr::internal::source_file_pos(__FUNCTION__,__FILE__,__LINE__)

#endif

