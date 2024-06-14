#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <string>

#include "spdlog/spdlog.h"

/*
 * 一些 common 的 error code
*/

// success
#define SUCC 0

// [1-1000) common error
#define UNEXPECTED_ERROR -1
#define POINTER_NULL_ERROR -2
#define FILE_NOT_EXIST_ERROR -3
#define KEY_NOT_FOUND_ERROR -4
#define OUT_OF_RANGE_ERROR -5
#define CARD_NOT_FOUND_ERROR -6
#define DECK_EMPTY_ERROR -7
#define PLAYER_NOT_FOUND_ERROR -8
#define CONDITION_NOT_SATISFY -9 // 发动条件不满足
#define NOT_IMPLEMENTED_ERROR -99

#define STRING(x) #x

inline std::string ErrString(const int rtn) {
  switch (rtn) {
    case UNEXPECTED_ERROR: {
      return STRING(UNEXPECTED_ERROR);
    }
    case POINTER_NULL_ERROR: {
      return STRING(POINTER_NULL_ERROR);
    }
    case FILE_NOT_EXIST_ERROR: {
      return STRING(FILE_NOT_EXIST_ERROR);
    }
    case KEY_NOT_FOUND_ERROR: {
      return STRING(KEY_NOT_FOUND_ERROR);
    }
    case OUT_OF_RANGE_ERROR: {
      return STRING(OUT_OF_RANGE_ERROR);
    }
    case CARD_NOT_FOUND_ERROR: {
      return STRING(CARD_NOT_FOUND_ERROR);
    }
    case DECK_EMPTY_ERROR: {
      return STRING(DECK_EMPTY_ERROR);
    }
    case PLAYER_NOT_FOUND_ERROR: {
      return STRING(PLAYER_NOT_FOUND_ERROR);
    }
    case NOT_IMPLEMENTED_ERROR: {
      return STRING(NOT_IMPLEMENTED_ERROR);
    }
  }
  return "Unknown Error";
}

#define ASSERT_IS_OF_TYPE(x, T)                      \
  static_assert(std::is_same<decltype(x), T>::value, \
                "rtn is not of type " STRING(T))
#define ASSERT_IS_INT(x) ASSERT_IS_OF_TYPE(x, int)


#define ERR_CHECK(rtn)                                                         \
  do {                                                                         \
    ASSERT_IS_INT(rtn);                                                        \
    if (rtn == SUCC) {                                                         \
      ;                                                                        \
    } else {                                                                   \
      for ((spdlog::error("{} : return error code: {}, {}",                    \
            __PRETTY_FUNCTION__, rtn, ErrString(rtn)));                        \
           ;)                                                                  \
        return rtn;                                                            \
    }                                                                          \
  } while (0)

#define ERR_CHECK_CTN(rtn)                                                     \
  ASSERT_IS_INT(rtn);                                                          \
  if (rtn == SUCC) {                                                           \
    ;                                                                          \
  } else {                                                                     \
    spdlog::error("{} : return error code: {}, {}",                            \
        __PRETTY_FUNCTION__, rtn, ErrString(rtn));                             \
      continue;                                                                \
  }

#endif