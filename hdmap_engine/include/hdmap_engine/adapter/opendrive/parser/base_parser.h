#ifndef HDMAP_ADAPTER_OPENDRIVE_BASE_PARSER_H_
#define HDMAP_ADAPTER_OPENDRIVE_BASE_PARSER_H_

#include <tinyxml2.h>

#include <memory>
#include <mutex>

#include "hdmap_engine/adapter/opendrive/element.h"
#include "hdmap_engine/common/pipeline.h"
#include "hdmap_engine/common/status.h"

namespace hdmap {
namespace opendrive {

class XmlParser {
 public:
  virtual ~XmlParser() = default;

  tinyxml2::XMLError XmlQueryBoolAttribute(const tinyxml2::XMLElement* xml_node,
                                           const std::string& name,
                                           bool& value) {
    return xml_node->QueryBoolAttribute(name.c_str(), &value);
  }

  tinyxml2::XMLError XmlQueryStringAttribute(
      const tinyxml2::XMLElement* xml_node, const std::string& name,
      std::string* value) {
    const char* val = xml_node->Attribute(name.c_str());
    if (nullptr == val) {
      return tinyxml2::XML_NO_ATTRIBUTE;
    }
    *value = val;
    return tinyxml2::XML_SUCCESS;
  }

  tinyxml2::XMLError XmlQueryIntAttribute(const tinyxml2::XMLElement* xml_node,
                                          const std::string& name, int* value) {
    return xml_node->QueryIntAttribute(name.c_str(), value);
  }

  tinyxml2::XMLError XmlQueryFloatAttribute(
      const tinyxml2::XMLElement* xml_node, const std::string& name,
      float* value) {
    return xml_node->QueryFloatAttribute(name.c_str(), value);
  }

  tinyxml2::XMLError XmlQueryDoubleAttribute(
      const tinyxml2::XMLElement* xml_node, const std::string& name,
      double* value) {
    return xml_node->QueryDoubleAttribute(name.c_str(), value);
  }

  const tinyxml2::XMLElement* XmlNextSiblingElement(
      const tinyxml2::XMLElement* element) {
    return element->NextSiblingElement(element->Name());
  }

  /**
   * @brief status is ok
   *
   * @return bool
   */
  bool ok() const { return status_.ok(); }

  /**
   * @brief update status
   *
   * @param error_code
   * @param msg
   */
  void UpdateStatus(ErrorCode error_code, const std::string& msg) {
    status_.set_error_code(error_code);
    status_.set_msg(msg);
  }

 private:
  /**
   * @brief engine status singleton
   */
  Status status_;
};

}  // namespace opendrive
}  // namespace hdmap

#endif  // HDMAP_ADAPTER_OPENDRIVE_BASE_PARSER_H_
