#ifndef HDMAP_ENGINE_ADAPTER_PARSER_H_
#define HDMAP_ENGINE_ADAPTER_PARSER_H_

#include "hdmap_engine/adapter/opendrive/parser/map_parser.h"
#include "hdmap_engine/common/param.h"
#include "hdmap_engine/common/pipeline.h"
#include "hdmap_engine/common/status.h"
#include "hdmap_engine/core/kdtree/kdtree.h"

namespace hdmap {

class ParseProcessor : public Processor {
 public:
  virtual ~ParseProcessor() = default;

 protected:
  bool Continue() const { return pipeline_data_->status->ok(); }

  void UpdateStatus(ErrorCode error_code, const std::string& msg) {
    pipeline_data_->status->set_error_code(error_code);
    pipeline_data_->status->set_msg(msg);
  }

  std::shared_ptr<PipelineLoading> pipeline_data_ = nullptr;
};

}  // namespace hdmap

#endif  // HDMAP_ENGINE_ADAPTER_PARSER_H_
