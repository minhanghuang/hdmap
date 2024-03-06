#ifndef HDMAP_ADAPTER_OPENDRIVE_PARSER_SECTION_H_
#define HDMAP_ADAPTER_OPENDRIVE_PARSER_SECTION_H_

#include "hdmap/adapter/opendrive/parser/base_parser.h"

namespace hdmap {
namespace opendrive {

class SectionXmlParser : public XmlParser {
 public:
  SectionXmlParser() = default;

  bool Parse(const tinyxml2::XMLElement* xml_section,
             element::LaneSection* ele_section);

 private:
  SectionXmlParser& ParseAttributes();

  SectionXmlParser& ParseLanesEle();

  SectionXmlParser& ParseLaneEle(const tinyxml2::XMLElement* xml_lane,
                                 element::Lane& ele_lane);

  SectionXmlParser& ParseLaneAttributes(const tinyxml2::XMLElement* xml_lane,
                                        element::Lane& ele_lane);

  SectionXmlParser& ParseLaneLinkEle(const tinyxml2::XMLElement* xml_lane,
                                     element::Lane& ele_lane);

  SectionXmlParser& ParseLaneWidthEle(const tinyxml2::XMLElement* xml_lane,
                                      element::Lane& ele_lane);

  SectionXmlParser& ParseLaneBorderEle(const tinyxml2::XMLElement* xml_lane,
                                       element::Lane& ele_lane);

  SectionXmlParser& ParseLaneRoadMarkEle(const tinyxml2::XMLElement* xml_lane,
                                         element::Lane& ele_lane);

  SectionXmlParser& ParseLaneSpeedEle(const tinyxml2::XMLElement* xml_lane,
                                      element::Lane& ele_lane);

  const tinyxml2::XMLElement* xml_section_;

  element::LaneSection* ele_section_;
};

}  // namespace opendrive
}  // namespace hdmap

#endif  // HDMAP_ADAPTER_OPENDRIVE_PARSER_SECTION_H_
