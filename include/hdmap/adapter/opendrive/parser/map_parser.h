#ifndef HDMAP_ADAPTER_OPENDRIVE_PARSER_MAP_H_
#define HDMAP_ADAPTER_OPENDRIVE_PARSER_MAP_H_

#include "hdmap/adapter/opendrive/parser/base_parser.h"
#include "hdmap/adapter/opendrive/parser/road_parser.h"

namespace hdmap {
namespace opendrive {

class MapXmlParser : public RoadXmlParser {
 public:
  MapXmlParser();
  bool Parse(const tinyxml2::XMLElement* xml_map, element::Map* ele_map);

 private:
  MapXmlParser& ParseJunctionElement();

  MapXmlParser& ParseRoadElement();

  const tinyxml2::XMLElement* xml_map_;

  element::Map* ele_map_;
};

}  // namespace opendrive
}  // namespace hdmap

#endif  // HDMAP_ADAPTER_OPENDRIVE_PARSER_MAP_H_
