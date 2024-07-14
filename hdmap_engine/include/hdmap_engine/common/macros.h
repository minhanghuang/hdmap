#ifndef HDMAP_ENGINE_COMMON_MACROS_H_
#define HDMAP_ENGINE_COMMON_MACROS_H_

#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

namespace hdmap {

#define ADD_MEMBER_BASIC_TYPE(type, var, value) \
 private:                                       \
  type var##_ = value;                          \
                                                \
 public:                                        \
  type var() const noexcept { return var##_; }  \
  void set_##var(const type var) noexcept { var##_ = var; }

#define ADD_MEMBER_COMPLEX_TYPE(type, var)                   \
 private:                                                    \
  type var##_;                                               \
                                                             \
 public:                                                     \
  const type& var() const noexcept { return var##_; }        \
  type* mutable_##var() noexcept { return &var##_; }         \
  void set_##var(const type& var) noexcept { var##_ = var; } \
  void set_##var(type&& var) noexcept { var##_ = std::move(var); }

#define ADD_MEMBER_SHAREDPTR_TYPE(type, var)                                  \
 private:                                                                     \
  std::shared_ptr<type> var##_;                                               \
                                                                              \
 public:                                                                      \
  std::shared_ptr<type const> var() const noexcept { return var##_; }         \
  std::shared_ptr<type> mutable_##var() noexcept { return var##_; }           \
  void set_##var(const std::shared_ptr<type>& var) noexcept { var##_ = var; } \
  void set_##var(std::shared_ptr<type>&& var) noexcept {                      \
    var##_ = std::move(var);                                                  \
  }

#define HDMAP_DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname&) = delete;           \
  classname& operator=(const classname&) = delete;

#define HDMAP_DECLARE_SINGLETON(classname)                            \
 public:                                                              \
  static std::shared_ptr<classname> Instance() {                      \
    static std::shared_ptr<classname> instance = nullptr;             \
    if (!instance) {                                                  \
      static std::once_flag flag;                                     \
      std::call_once(flag, [&] { instance.reset(new classname()); }); \
    }                                                                 \
    return instance;                                                  \
  }                                                                   \
                                                                      \
 private:                                                             \
  classname();                                                        \
  HDMAP_DISALLOW_COPY_AND_ASSIGN(classname)

}  // namespace hdmap

#endif  // HDMAP_ENGINE_COMMON_MACROS_H_
