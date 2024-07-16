#ifndef HDMAP_COMMON_UTIL_H_
#define HDMAP_COMMON_UTIL_H_

#include <boost/algorithm/string.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace hdmap {
namespace common {

std::vector<std::string> Split(const std::string& input,
                               const std::string& delimiter);

/**
 * @brief Generates a random UUID (Universally Unique Identifier) version 4.
 *
 * This function creates a UUID using random numbers. The generated UUID
 * follows the format: xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
 * where x is any hexadecimal digit and y is one of 8, 9, A, or B.
 *
 * @return std::string A string representation of the generated UUID.
 */
std::string GenerateUuid();

/**
 * @brief Returns the minimum value among the provided arguments.
 *
 * This function takes a variable number of arguments and returns the minimum
 * value among them. It uses a fold expression available in C++17 and later
 * to efficiently compute the minimum value.
 *
 * @tparam T The type of the arguments. All arguments must be of the same type.
 * @tparam Args The types of the remaining arguments (variadic template).
 * @param first The first argument.
 * @param args The remaining arguments.
 * @return The minimum value among the provided arguments.
 */
template <typename T, typename... Args>
auto MinValue(const T& first, const Args&... args) ->
    typename std::common_type<T, Args...>::type {
  using CommonType = typename std::common_type<T, Args...>::type;
  return (std::min)(
      {static_cast<CommonType>(first), static_cast<CommonType>(args)...});
}

std::string GetLaneIdByPointId(const std::string& point_id);

/**
 * @brief 计算偏离点
 *
 * @tparam T element::Point
 * @param point 基准点
 * @param lateral_offset 横向偏离距离
 * @return 偏离点
 */
template <typename T>
T GetOffsetPoint(const T& point, double lateral_offset) {
  const double x = -std::sin(point.heading());
  const double y = std::cos(point.heading());
  T offset_point = point;
  offset_point.set_x(offset_point.x() + lateral_offset * x);
  offset_point.set_y(offset_point.y() + lateral_offset * y);
  return offset_point;
}

/**
 * @brief 容器排序
 *
 * @tparam T Poloy3派生类
 * @param items Poloy3 vector;
 * @param asc 升序/降序
 */
template <typename T>
void VectorSortPoloy3(std::vector<T>* items, bool asc = true) {
  std::sort(items->begin(), items->end(), [asc](const T& t1, const T& t2) {
    return asc ? t1.s() < t2.s() : t1.s() > t2.s();
  });
}

/**
 * @brief 获取目标值左边的元素(包括目标值)
 *
 * @tparam T1 element::Poloy3
 * @tparam T2 number
 * @param items ascending sequence
 * @param target target value
 * @return sequence index or -1
 */
template <typename T1, typename T2>
int GetGeValuePoloy3(const std::vector<T1>& items, T2 target) {
  if (items.empty() || target < items.at(0).s()) return -1;
  for (int i = items.size() - 1; i >= 0; i--) {
    if (target >= items.at(i).s()) return i;
  }
  return -1;
}

template <typename T1, typename T2>
int GetGtValuePoloy3(const std::vector<T1>& items, T2 target) {
  if (items.empty() || target < items.at(0).s()) return -1;
  if (target < items.at(0).s()) return -1;
  for (int i = items.size() - 1; i >= 0; i--) {
    if (target > items.at(i).s()) return i;
  }
  return 0;
}

template <typename T1, typename T2>
int GetGePtrPoloy3(const std::vector<T1>& items, T2 target) {
  if (items.empty() || target < items.at(0)->s()) return -1;
  for (int i = items.size() - 1; i >= 0; i--) {
    if (target >= items.at(i)->s()) return i;
  }
  return -1;
}

template <typename T1, typename T2>
int GetGtPtrPoloy3(const std::vector<T1>& items, T2 target) {
  if (items.empty() || target < items.at(0)->s()) return -1;
  if (target < items.at(0)->s()) return -1;
  for (int i = items.size() - 1; i >= 0; i--) {
    if (target > items.at(i)->s()) return i;
  }
  return 0;
}

}  // namespace common
}  // namespace hdmap

#endif  // HDMAP_COMMON_UTIL_H_
