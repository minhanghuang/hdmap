#ifndef HDMAP_COMMON_PIPELINE_H_
#define HDMAP_COMMON_PIPELINE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hdmap_engine/adapter/opendrive/element.h"
#include "hdmap_engine/algo/kdtree/kdtree.h"
#include "hdmap_engine/common/param.h"
#include "hdmap_engine/common/status.h"
#include "hdmap_engine/geometry.h"

namespace hdmap {

struct PipelineData {
 public:
  virtual ~PipelineData() = default;

  Param::Ptr param;

  Status::Ptr status;

  kdtree::KDTree::Ptr kdtree;
};

struct PipelineLoading : public PipelineData {
 public:
  PipelineLoading() = default;

  geometry::Map::Ptr map;

  opendrive::element::Map::Ptr opendrive_element_map;
};

class Processor {
 public:
  virtual ~Processor() = default;

  virtual bool process(std::shared_ptr<PipelineData>) = 0;

  const std::string& msg() const { return msg_; }

 protected:
  void AddMsg(const std::string& msg) {
    msg_ += "[" + name_ + "]  " + msg + "\n";
  }

  void set_name(const std::string& name) { name_ = name; }

 private:
  std::string name_;

  std::string msg_;
};

class Pipeline final {
 public:
  void AddProcessor(std::shared_ptr<Processor> processor) {
    processors_.emplace_back(std::move(processor));
  }

  void execute(std::shared_ptr<PipelineData> pipeline_data) {
    for (const auto& processor : processors_) {
      if (!processor->process(pipeline_data)) {
        report_ += processor->msg();
        ok_ = false;
        break;
      }
      report_ += processor->msg();
    }
  }

  const std::string& report() const { return report_; }

  bool ok() const { return ok_; }

 private:
  std::vector<std::shared_ptr<Processor>> processors_;

  std::string report_;

  bool ok_ = true;
};

}  // namespace hdmap

#endif  // HDMAP_COMMON_PIPELINE_H_
