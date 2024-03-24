#ifndef HDMAP_ADAPTER_CONVERTOR_H_
#define HDMAP_ADAPTER_CONVERTOR_H_

#include "hdmap/adapter/opendrive/parser/map_parser.h"
#include "hdmap/algo/kdtree/kdtree.h"
#include "hdmap/common/param.h"
#include "hdmap/common/pipeline.h"
#include "hdmap/common/status.h"

namespace hdmap {

class ConvertProcessor : public Processor {
 public:
  virtual ~ConvertProcessor() = default;

 protected:
  bool Continue() const { return pipeline_data_->status->ok(); }

  void UpdateStatus(ErrorCode error_code, const std::string& msg) {
    pipeline_data_->status->set_error_code(error_code);
    pipeline_data_->status->set_msg(msg);
  }

  std::shared_ptr<PipelineLoading> pipeline_data_ = nullptr;
};

}  // namespace hdmap

#endif  // HDMAP_ADAPTER_CONVERTOR_H_
