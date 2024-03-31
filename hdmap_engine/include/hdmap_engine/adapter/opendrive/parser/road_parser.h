#ifndef HDMAP_ADAPTER_OPENDRIVE_PARSER_ROAD_H_
#define HDMAP_ADAPTER_OPENDRIVE_PARSER_ROAD_H_

#include "hdmap_engine/adapter/opendrive/parser/base_parser.h"
#include "hdmap_engine/adapter/opendrive/parser/section_parser.h"

namespace hdmap {
namespace opendrive {

class RoadXmlParser : public SectionXmlParser {
 public:
  RoadXmlParser() = default;

  bool Parse(const tinyxml2::XMLElement* xml_road, element::Road* ele_road);

 private:
  RoadXmlParser& ParseAttributes();

  RoadXmlParser& ParseLinkElement();

  RoadXmlParser& ParseTypeElement();

  RoadXmlParser& ParsePlanViewElement();

  RoadXmlParser& ParseLanesElement();

  RoadXmlParser& GenerateRoad();

  const tinyxml2::XMLElement* xml_road_;

  element::Road* ele_road_;
};

}  // namespace opendrive
}  // namespace hdmap

#endif  // HDMAP_ADAPTER_OPENDRIVE_PARSER_ROAD_H_
