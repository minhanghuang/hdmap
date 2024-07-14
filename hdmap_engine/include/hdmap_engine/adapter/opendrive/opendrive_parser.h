#ifndef HDMAP_ENGINE_ADAPTER_OPENDRIVE_PARSER_H_
#define HDMAP_ENGINE_ADAPTER_OPENDRIVE_PARSER_H_

#include "hdmap_engine/adapter/opendrive/parser/header_parser.h"
#include "hdmap_engine/adapter/opendrive/parser/map_parser.h"
#include "hdmap_engine/adapter/parser.h"

namespace hdmap {

class OpenDriveParser : public ParseProcessor {
 public:
  OpenDriveParser() = default;

  virtual bool process(std::shared_ptr<PipelineData>) override;

 private:
  OpenDriveParser& CheckIn(std::shared_ptr<PipelineData>);

  OpenDriveParser& Parse();

  bool Finish();
};

}  // namespace hdmap

#endif  // HDMAP_ENGINE_ADAPTER_OPENDRIVE_PARSER_H_
