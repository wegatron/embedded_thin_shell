#ifndef _ASSERTEXT_H_
#define _ASSERTEXT_H_
/**
 * this is some extetions for the assert command.
 */

#include <assert.h>
#include <string>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <iostream>

/**
 * If NDEBUG is defined, do nothing.
 */
#ifdef	NDEBUG
# define assert_eq(value_a,value_b)		(__ASSERT_VOID_CAST (0))
# define assert_ne(value_a,value_b)		(__ASSERT_VOID_CAST (0))
# define assert_ge(value_a,value_b)		(__ASSERT_VOID_CAST (0))
# define assert_gt(value_a,value_b)		(__ASSERT_VOID_CAST (0))
# define assert_le(value_a,value_b)		(__ASSERT_VOID_CAST (0))
# define assert_lt(value_a,value_b)		(__ASSERT_VOID_CAST (0))
# define assert_in(value_a,min,max)		(__ASSERT_VOID_CAST (0))

#else /* Not NDEBUG.  */

// define a new assert micro to throw expections instead of abort
#define new_assert(e) ((void) ((e) ? 0 : my_assert (#e, __FILE__, __LINE__)))
#define my_assert( e, file, line ) ( throw std::runtime_error(\
   std::string(file)+":"+boost::lexical_cast<std::string>(line)+": failed assertion "+e))

// assert "value_a == value_b", if not, print the values and abort.
#define assert_eq(value_a, value_b)                                     \
  if (value_a != value_b)                                               \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;	\
    std::cout << __STRING(value_b) <<" = " << value_b<< std::endl;	\
    new_assert(value_a == value_b);                                     \
  }														

// assert "value_a != value_b", if not, print the values and abort.
#define assert_ne(value_a, value_b)                                     \
  if (value_a == value_b)                                               \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;	\
    std::cout << __STRING(value_b) <<" = " << value_b<< std::endl;	\
    new_assert(value_a != value_b);                                     \
  }														

// assert "value_a >= value_b", if not, print the values and abort.
#define assert_ge(value_a, value_b)                                     \
  if (value_a < value_b)                                                \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;	\
    std::cout << __STRING(value_b) <<" = " << value_b<< std::endl;	\
    new_assert(value_a >= value_b);                                     \
  }														

// assert "value_a > value_b", if not, print the values and abort.
#define assert_gt(value_a, value_b)                                     \
  if (value_a <= value_b)                                               \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;	\
    std::cout << __STRING(value_b) <<" = " << value_b<< std::endl;	\
    new_assert(value_a > value_b);                                      \
  }														

// assert "value_a >= value_b", if not, print the values and abort.
#define assert_le(value_a, value_b)                                     \
  if (value_a > value_b)                                                \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;	\
    std::cout << __STRING(value_b) <<" = " << value_b<< std::endl;	\
    new_assert(value_a <= value_b);                                     \
  }														

// assert "value_a > value_b", if not, print the values and abort.
#define assert_lt(value_a, value_b)                                     \
  if (value_a >= value_b)                                               \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;	\
    std::cout << __STRING(value_b) <<" = " << value_b<< std::endl;	\
    new_assert(value_a < value_b);                                      \
  }														

// assert "value_a in [min,max]", if not, print the values and abort.
#define assert_in(value_a, min, max)                                    \
  if ( !(value_a >= min && value_a <= max) )                            \
  {                                                                     \
    std::cout << __STRING(value_a) <<" = " << value_a<< std::endl;      \
    std::cout << __STRING(min) <<" = " << min << std::endl;             \
    std::cout << __STRING(max) <<" = " << max << std::endl;             \
    new_assert(value_a >= min && value_a <= max);                       \
  }
#endif /* NDEBUG.  */

#endif /* _ASSERTEXT_H_ */
