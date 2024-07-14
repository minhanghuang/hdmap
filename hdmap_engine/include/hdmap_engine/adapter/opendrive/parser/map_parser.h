#ifndef HDMAP_ENGINE_ADAPTER_OPENDRIVE_PARSER_MAP_H_
#define HDMAP_ENGINE_ADAPTER_OPENDRIVE_PARSER_MAP_H_

#include "hdmap_engine/adapter/opendrive/parser/base_parser.h"
#include "hdmap_engine/adapter/opendrive/parser/road_parser.h"

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

#endif  // HDMAP_ENGINE_ADAPTER_OPENDRIVE_PARSER_MAP_H_
