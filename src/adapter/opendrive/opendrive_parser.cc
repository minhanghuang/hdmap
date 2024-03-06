#include "hdmap/adapter/opendrive/opendrive_parser.h"

namespace hdmap {

bool OpenDriveParser::process(std::shared_ptr<PipelineData> pipeline_data) {
  return CheckIn(pipeline_data).Parse().Finish();
}

OpenDriveParser& OpenDriveParser::CheckIn(
    std::shared_ptr<PipelineData> pipeline_data) {
  if (nullptr == pipeline_data) {
    UpdateStatus(ErrorCode::kInitError, "");
    return *this;
  }
  pipeline_data_ = std::static_pointer_cast<PipelineLoading>(pipeline_data);
  if (nullptr == pipeline_data_) {
    UpdateStatus(ErrorCode::XML_ROAD_ELEMENT_ERROR, "");
    return *this;
  }
  if (nullptr == pipeline_data_->opendrive_element_map ||
      nullptr == pipeline_data_->param || nullptr == pipeline_data_->status ||
      nullptr == pipeline_data_->map || nullptr == pipeline_data_->kdtree ||
      pipeline_data_->param->file_path().empty()) {
    UpdateStatus(ErrorCode::kInitError, "");
    return *this;
  }
  return *this;
}

OpenDriveParser& OpenDriveParser::Parse() {
  if (!Continue()) return *this;

  // get opendrive file element
  tinyxml2::XMLDocument xml_doc;
  xml_doc.LoadFile(pipeline_data_->param->file_path().c_str());
  if (xml_doc.Error()) {
    UpdateStatus(ErrorCode::XML_ROAD_ELEMENT_ERROR, "OpenDRIVE File Exection.");
    return *this;
  }
  tinyxml2::XMLElement* xml_root = xml_doc.RootElement();

  // parse opendriv element
  opendrive::HeaderXmlParser header_parser;
  opendrive::MapXmlParser map_parser;
  if (!header_parser.Parse(xml_root,
                           pipeline_data_->opendrive_element_map.get()) ||
      !map_parser.Parse(xml_root,
                        pipeline_data_->opendrive_element_map.get())) {
    UpdateStatus(ErrorCode::XML_ROAD_ELEMENT_ERROR, "");
    return *this;
  }
  return *this;
}

bool OpenDriveParser::Finish() {
  if (!Continue()) return false;
  return Continue();
}

}  // namespace hdmap
