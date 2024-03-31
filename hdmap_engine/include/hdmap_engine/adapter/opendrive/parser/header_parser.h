#ifndef HDMAP_ADAPTER_OPENDRIVE_PARSER_HEADER_H_
#define HDMAP_ADAPTER_OPENDRIVE_PARSER_HEADER_H_

#include "hdmap_engine/adapter/opendrive/parser/base_parser.h"

namespace hdmap {
namespace opendrive {

class HeaderXmlParser : public XmlParser {
 public:
  HeaderXmlParser() = default;
  bool Parse(const tinyxml2::XMLElement* xml_map, element::Map* ele_map);

 private:
  HeaderXmlParser& ParseHeaderElement();

  const tinyxml2::XMLElement* xml_map_;

  element::Map* ele_map_;
};

}  // namespace opendrive
}  // namespace hdmap

#endif  // HDMAP_ADAPTER_OPENDRIVE_PARSER_HEADER_H_
