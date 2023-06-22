#ifndef OPENDRIVE_ENGINE_SERVER_MSGS_H_
#define OPENDRIVE_ENGINE_SERVER_MSGS_H_

#include <cactus/macros.h>

#include <nlohmann/json.hpp>
#include <vector>

namespace opendrive {
namespace engine {
namespace server {

using Json = nlohmann::json;
using JsonValueType = nlohmann::json::value_t;
using RequiredKeys = std::unordered_map<std::string, JsonValueType>;

namespace msgs {

class Message {
 public:
  virtual ~Message() = default;
  Message() = default;
  virtual Json ToJson() const = 0;
};

class Point : public Message {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, x, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, y, 0);

 public:
  Point() = default;
  Point(double x, double y) : x_(x), y_(y) {}
  Json ToJson() const override { return {{"x", x()}, {"y", y()}}; }
};

class Line : public Message {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::vector<Point>, pts);

 public:
  Line() = default;
  Json ToJson() const override {
    Json json;
    for (const auto& p : pts()) {
      json.emplace_back(p.ToJson());
    }
    return json;
  }
};

class Lane : public Message {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Line, left_boundary);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Line, right_boundary);

 public:
  Lane() = default;
  Json ToJson() const override {
    return {{"left_boundary", left_boundary().ToJson()},
            {"right_boundary", right_boundary().ToJson()}};
  }
};

class Lanes : public Message {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::vector<Lane>, lanes);

 public:
  Lanes() = default;
  Json ToJson() const override {
    Json json;
    for (const auto& lane : lanes()) {
      json.emplace_back(lane.ToJson());
    }
    return json;
  }
};

}  // namespace msgs
}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_MSGS_H_
