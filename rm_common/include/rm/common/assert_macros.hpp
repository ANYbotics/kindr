/*
 * assert_macros.hpp
 *
 *  Created on: Nov 20, 2013
 *      Author: gech
 */

#ifndef RM_ASSERT_MACROS_HPP_
#define RM_ASSERT_MACROS_HPP_


#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include "source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define RM_DEFINE_EXCEPTION(exceptionName, exceptionParent)       \
  class exceptionName : public exceptionParent {            \
  public:                               \
  exceptionName(const char * message) : exceptionParent(message) {}   \
  exceptionName(std::string const & message) : exceptionParent(message) {} \
  virtual ~exceptionName() throw() {}                 \
  };


namespace rm {
namespace common {
  namespace internal {

    template<typename RM_EXCEPTION_T>
    inline void rm_throw_exception(std::string const & exceptionType, rm::source_file_pos sfp, std::string const & message)
    {
      std::stringstream RM_assert_stringstream;
#ifdef _WIN32
      // I have no idea what broke this on Windows but it doesn't work with the << operator.
      RM_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#else
      RM_assert_stringstream << exceptionType <<  sfp << " " << message;
#endif
      throw(RM_EXCEPTION_T(RM_assert_stringstream.str()));
    }

    template<typename RM_EXCEPTION_T>
    inline void rm_throw_exception(std::string const & exceptionType, std::string const & function, std::string const & file,
                   int line, std::string const & message)
    {
      rm_throw_exception<RM_EXCEPTION_T>(exceptionType, rm::source_file_pos(function,file,line),message);
    }


  } // namespace internal

  template<typename RM_EXCEPTION_T>
  inline void rm_assert_throw(bool assert_condition, std::string message, rm::source_file_pos sfp) {
    if(!assert_condition)
      {
    internal::rm_throw_exception<RM_EXCEPTION_T>("", sfp,message);
      }
  }


} // namespace common
} // namespace rm




#define RM_THROW(exceptionType, message) {                \
    std::stringstream RM_assert_stringstream;             \
    RM_assert_stringstream << message;                  \
    rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, RM_assert_stringstream.str()); \
  }


#define RM_THROW_SFP(exceptionType, SourceFilePos, message){      \
    std::stringstream RM_assert_stringstream;             \
    RM_assert_stringstream << message;                  \
    rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, RM_assert_stringstream.str()); \
  }

#define RM_ASSERT_TRUE(exceptionType, condition, message)       \
  if(!(condition))                            \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_FALSE(exceptionType, condition, message)        \
  if((condition))                           \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_LT(exceptionType, value, upperBound, message)     \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_GE(exceptionType, value, lowerBound, message)     \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_LE(exceptionType, value, upperBound, message)     \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_GT(exceptionType, value, lowerBound, message)     \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_EQ(exceptionType, value, testValue, message)      \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_NE(exceptionType, value, testValue, message)      \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#ifndef NDEBUG

#define RM_THROW_DBG(exceptionType, message){             \
    std::stringstream RM_assert_stringstream;             \
    RM_assert_stringstream << message;                  \
    rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, RM_assert_stringstream.str()); \
  }



#define RM_ASSERT_TRUE_DBG(exceptionType, condition, message)     \
  if(!(condition))                            \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_FALSE_DBG(exceptionType, condition, message)      \
  if((condition))                           \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, RM_assert_stringstream.str()); \
    }


#define RM_ASSERT_DBG_RE( condition, message) RM_ASSERT_DBG(std::runtime_error, condition, message)

#define RM_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_LT_DBG(exceptionType, value, upperBound, message)   \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)   \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_LE_DBG(exceptionType, value, upperBound, message)   \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }

#define RM_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)   \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_EQ_DBG(exceptionType, value, testValue, message)    \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }


#define RM_ASSERT_NE_DBG(exceptionType, value, testValue, message)    \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }



#define RM_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream RM_assert_stringstream;             \
      RM_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      rm::common::internal::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,RM_assert_stringstream.str()); \
    }


#define RM_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define RM_OUT(X)
#define RM_THROW_DBG(exceptionType, message)
#define RM_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define RM_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define RM_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define RM_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define RM_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define RM_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define RM_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define RM_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define RM_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define RM_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)
#endif

#endif /* RM_ASSERT_MACROS_HPP_ */
