#ifndef SM_SOURCE_FILE_POS_HPP
#define SM_SOURCE_FILE_POS_HPP

#include <string>
#include <iostream>
#include <sstream>
// A class and macro that gives you the current file position.

namespace rm {

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

}// namespace rm

inline std::ostream & operator<<(std::ostream & out, const rm::source_file_pos & sfp)
{
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}


#define RM_SOURCE_FILE_POS rm::source_file_pos(__FUNCTION__,__FILE__,__LINE__)

#endif

