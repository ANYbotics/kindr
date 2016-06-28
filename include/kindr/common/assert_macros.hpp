/*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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
#ifndef KINDR_ASSERT_MACROS_HPP_
#define KINDR_ASSERT_MACROS_HPP_


#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include "source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define KINDR_DEFINE_EXCEPTION(exceptionName, exceptionParent)       \
  class exceptionName : public exceptionParent {            \
  public:                               \
  exceptionName(const char * message) : exceptionParent(message) {}   \
  exceptionName(std::string const & message) : exceptionParent(message) {} \
  virtual ~exceptionName() throw() {}                 \
  };


namespace kindr {
  namespace internal {

    template<typename KINDR_EXCEPTION_T>
    inline void kindr_throw_exception(std::string const & exceptionType, kindr::internal::source_file_pos sfp, std::string const & message)
    {
      std::stringstream kindr_assert_stringstream;
#ifdef _WIN32
      // I have no idea what broke this on Windows but it doesn't work with the << operator.
      kindr_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#else
      kindr_assert_stringstream << exceptionType <<  sfp.toString() << std::string{" "} << message;
#endif
      throw(KINDR_EXCEPTION_T(kindr_assert_stringstream.str()));
    }

    template<typename KINDR_EXCEPTION_T>
    inline void kindr_throw_exception(std::string const & exceptionType, std::string const & function, std::string const & file,
                   int line, std::string const & message)
    {
      kindr_throw_exception<KINDR_EXCEPTION_T>(exceptionType, kindr::internal::source_file_pos(function,file,line),message);
    }


  } // namespace internal

  template<typename KINDR_EXCEPTION_T>
  inline void kindr_assert_throw(bool assert_condition, std::string message, kindr::internal::source_file_pos sfp) {
    if(!assert_condition)
      {
    internal::kindr_throw_exception<KINDR_EXCEPTION_T>("", sfp,message);
      }
  }



} // namespace kindr




#define KINDR_THROW(exceptionType, message) {                \
    std::stringstream kindr_assert_stringstream;             \
    kindr_assert_stringstream << message;                  \
    kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, kindr_assert_stringstream.str()); \
  }


#define KINDR_THROW_SFP(exceptionType, SourceFilePos, message){      \
    std::stringstream kindr_assert_stringstream;             \
    kindr_assert_stringstream << message;                  \
    kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, kindr_assert_stringstream.str()); \
  }

#define KINDR_ASSERT_TRUE(exceptionType, condition, message)       \
  if(!(condition))                            \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_FALSE(exceptionType, condition, message)        \
  if((condition))                           \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_LT(exceptionType, value, upperBound, message)     \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_GE(exceptionType, value, lowerBound, message)     \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_LE(exceptionType, value, upperBound, message)     \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_GT(exceptionType, value, lowerBound, message)     \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_EQ(exceptionType, value, testValue, message)      \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_NE(exceptionType, value, testValue, message)      \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#ifndef NDEBUG

#define KINDR_THROW_DBG(exceptionType, message){             \
    std::stringstream kindr_assert_stringstream;             \
    kindr_assert_stringstream << message;                  \
    kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, kindr_assert_stringstream.str()); \
  }



#define KINDR_ASSERT_TRUE_DBG(exceptionType, condition, message)     \
  if(!(condition))                            \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_FALSE_DBG(exceptionType, condition, message)      \
  if((condition))                           \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, kindr_assert_stringstream.str()); \
    }


#define KINDR_ASSERT_DBG_RE( condition, message) KINDR_ASSERT_DBG(std::runtime_error, condition, message)

#define KINDR_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_LT_DBG(exceptionType, value, upperBound, message)   \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)   \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_LE_DBG(exceptionType, value, upperBound, message)   \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }

#define KINDR_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)   \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_EQ_DBG(exceptionType, value, testValue, message)    \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }


#define KINDR_ASSERT_NE_DBG(exceptionType, value, testValue, message)    \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }



#define KINDR_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream kindr_assert_stringstream;             \
      kindr_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      kindr::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }


#define KINDR_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define KINDR_OUT(X)
#define KINDR_THROW_DBG(exceptionType, message)
#define KINDR_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define KINDR_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define KINDR_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define KINDR_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define KINDR_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define KINDR_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define KINDR_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define KINDR_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define KINDR_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define KINDR_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)
#endif

#endif /* KINDR_ASSERT_MACROS_HPP_ */
