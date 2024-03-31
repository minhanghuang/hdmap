#include "hdmap_engine/adapter/opendrive/parser/header_parser.h"

namespace hdmap {
namespace opendrive {

bool HeaderXmlParser::Parse(const tinyxml2::XMLElement* xml_map,
                            element::Map* ele_map) {
  xml_map_ = xml_map;
  ele_map_ = ele_map;
  if (!xml_map_ || !ele_map_) {
    UpdateStatus(ErrorCode::XML_HEADER_ELEMENT_ERROR, "Input is null.");
    return false;
  }

  ParseHeaderElement();
  return true;
}

HeaderXmlParser& HeaderXmlParser::ParseHeaderElement() {
  if (!ok()) return *this;
  // eq 1
  const tinyxml2::XMLElement* xml_header =
      xml_map_->FirstChildElement("header");
  if (!xml_header) {
    UpdateStatus(ErrorCode::XML_HEADER_ELEMENT_ERROR,
                 "HEADER ELEMENT IS NULL.");
    return *this;
  }
  XmlQueryStringAttribute(xml_header, "revMajor",
                          ele_map_->mutable_header()->mutable_rev_major());
  XmlQueryStringAttribute(xml_header, "revMinor",
                          ele_map_->mutable_header()->mutable_rev_minor());
  XmlQueryStringAttribute(xml_header, "name",
                          ele_map_->mutable_header()->mutable_name());
  XmlQueryStringAttribute(xml_header, "version",
                          ele_map_->mutable_header()->mutable_version());
  XmlQueryStringAttribute(xml_header, "vendor",
                          ele_map_->mutable_header()->mutable_vendor());
  XmlQueryStringAttribute(xml_header, "date",
                          ele_map_->mutable_header()->mutable_date());
  double north = ele_map_->mutable_header()->north();
  double south = ele_map_->mutable_header()->south();
  double west = ele_map_->mutable_header()->west();
  double east = ele_map_->mutable_header()->east();
  XmlQueryDoubleAttribute(xml_header, "north", &north);
  ele_map_->mutable_header()->set_north(north);
  XmlQueryDoubleAttribute(xml_header, "south", &south);
  ele_map_->mutable_header()->set_south(south);
  XmlQueryDoubleAttribute(xml_header, "west", &west);
  ele_map_->mutable_header()->set_west(west);
  XmlQueryDoubleAttribute(xml_header, "east", &east);
  ele_map_->mutable_header()->set_east(east);
  return *this;
}

}  // namespace opendrive
}  // namespace hdmap
