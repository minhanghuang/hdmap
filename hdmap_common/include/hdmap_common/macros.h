#ifndef HDMAP_COMMON_MACROS_H_
#define HDMAP_COMMON_MACROS_H_

#include <memory>
#include <mutex>

#define HDMAP_DECLARE_SINGLETON(classname)                                \
 public:                                                                  \
  using Ptr = classname*;                                                 \
  static Ptr GetInstance() {                                              \
    static Ptr instance = nullptr;                                        \
    if (!instance) {                                                      \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  classname(const classname&) = delete;                                   \
  classname& operator=(const classname&) = delete;

#endif  // HDMAP_COMMON_MACROS_H_
