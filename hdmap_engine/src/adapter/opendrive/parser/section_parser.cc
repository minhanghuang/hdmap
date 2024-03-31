#include "hdmap_engine/adapter/opendrive/parser/section_parser.h"

namespace hdmap {
namespace opendrive {

bool SectionXmlParser::Parse(const tinyxml2::XMLElement* xml_section,
                             element::LaneSection* ele_section) {
  xml_section_ = xml_section;
  ele_section_ = ele_section;
  if (!xml_section_ || !ele_section_) {
    UpdateStatus(ErrorCode::XML_LANES_SECTION_ELEMENT_ERROR, "Input is null.");
    return false;
  }
  ParseAttributes().ParseLanesEle();
  return true;
}

SectionXmlParser& SectionXmlParser::ParseAttributes() {
  if (!ok()) return *this;
  double s = ele_section_->start_position();
  double e = ele_section_->end_position();
  XmlQueryDoubleAttribute(xml_section_, "s", &s);
  ele_section_->set_start_position(s);
  XmlQueryDoubleAttribute(xml_section_, "s", &e);
  ele_section_->set_end_position(e);
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLanesEle() {
  if (!ok()) return *this;
  std::vector<std::string> lanes_type{"left", "center", "right"};
  for (const auto& type : lanes_type) {
    const tinyxml2::XMLElement* curr_xml_lanes =
        xml_section_->FirstChildElement(type.c_str());
    if (!curr_xml_lanes) {
      continue;
    }
    const tinyxml2::XMLElement* curr_xml_lane =
        curr_xml_lanes->FirstChildElement("lane");
    while (curr_xml_lane) {
      element::Lane lane;
      this->ParseLaneEle(curr_xml_lane, lane);
      if ("left" == type) {
        ele_section_->mutable_left()->mutable_lanes()->emplace_back(lane);
      } else if ("center" == type) {
        ele_section_->mutable_center()->mutable_lanes()->emplace_back(lane);
      } else if ("right" == type) {
        ele_section_->mutable_right()->mutable_lanes()->emplace_back(lane);
      }
      curr_xml_lane = XmlNextSiblingElement(curr_xml_lane);
    }
  }
  /// sort left lanes
  std::sort(ele_section_->mutable_left()->mutable_lanes()->begin(),
            ele_section_->mutable_left()->mutable_lanes()->end(),
            [](const element::Lane& l1, const element::Lane& l2) {
              return l1.attribute().id() < l2.attribute().id();
            });
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneEle(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  ParseLaneAttributes(xml_lane, ele_lane)
      .ParseLaneWidthEle(xml_lane, ele_lane)
      .ParseLaneLinkEle(xml_lane, ele_lane)
      .ParseLaneBorderEle(xml_lane, ele_lane)
      .ParseLaneRoadMarkEle(xml_lane, ele_lane)
      .ParseLaneSpeedEle(xml_lane, ele_lane);
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneAttributes(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  std::string lane_type;
  std::string lane_level;
  int id = ele_lane.mutable_attribute()->id();
  XmlQueryIntAttribute(xml_lane, "id", &id);
  ele_lane.mutable_attribute()->set_id(id);
  XmlQueryStringAttribute(xml_lane, "type", &lane_type);
  XmlQueryStringAttribute(xml_lane, "level", &lane_level);
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneLinkEle(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  const tinyxml2::XMLElement* xml_lane_link =
      xml_lane->FirstChildElement("link");
  if (xml_lane_link) {
    element::Id id = 0;
    const tinyxml2::XMLElement* curr_xml_lane_link_predecessor =
        xml_lane_link->FirstChildElement("predecessor");
    while (curr_xml_lane_link_predecessor) {
      XmlQueryIntAttribute(curr_xml_lane_link_predecessor, "id", &id);
      ele_lane.mutable_link()->mutable_predecessors()->emplace_back(id);
      curr_xml_lane_link_predecessor =
          XmlNextSiblingElement(curr_xml_lane_link_predecessor);
    }
    const tinyxml2::XMLElement* curr_xml_lane_link_successor =
        xml_lane_link->FirstChildElement("successor");
    while (curr_xml_lane_link_successor) {
      XmlQueryIntAttribute(curr_xml_lane_link_successor, "id", &id);
      ele_lane.mutable_link()->mutable_successors()->emplace_back(id);
      curr_xml_lane_link_successor =
          XmlNextSiblingElement(curr_xml_lane_link_successor);
    }
  }

  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneWidthEle(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  const tinyxml2::XMLElement* curr_xml_width =
      xml_lane->FirstChildElement("width");
  double s;
  double a;
  double b;
  double c;
  double d;
  while (curr_xml_width) {
    element::LaneWidth lane_width;
    s = lane_width.s();
    a = lane_width.a();
    b = lane_width.b();
    c = lane_width.c();
    d = lane_width.d();
    XmlQueryDoubleAttribute(curr_xml_width, "sOffset", &s);
    lane_width.set_s(s);
    XmlQueryDoubleAttribute(curr_xml_width, "a", &a);
    lane_width.set_a(a);
    XmlQueryDoubleAttribute(curr_xml_width, "b", &b);
    lane_width.set_b(b);
    XmlQueryDoubleAttribute(curr_xml_width, "c", &c);
    lane_width.set_c(c);
    XmlQueryDoubleAttribute(curr_xml_width, "d", &d);
    lane_width.set_d(d);
    ele_lane.mutable_widths()->emplace_back(lane_width);
    curr_xml_width = XmlNextSiblingElement(curr_xml_width);
  }
  common::VectorSortPoloy3(ele_lane.mutable_widths());
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneBorderEle(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  const tinyxml2::XMLElement* curr_xml_border =
      xml_lane->FirstChildElement("border");
  double s;
  double a;
  double b;
  double c;
  double d;
  while (curr_xml_border) {
    element::LaneBorder lane_border;
    s = lane_border.s();
    a = lane_border.a();
    b = lane_border.b();
    c = lane_border.c();
    d = lane_border.d();
    XmlQueryDoubleAttribute(curr_xml_border, "sOffset", &s);
    lane_border.set_s(s);
    XmlQueryDoubleAttribute(curr_xml_border, "a", &a);
    lane_border.set_a(a);
    XmlQueryDoubleAttribute(curr_xml_border, "b", &b);
    lane_border.set_b(b);
    XmlQueryDoubleAttribute(curr_xml_border, "c", &c);
    lane_border.set_c(c);
    XmlQueryDoubleAttribute(curr_xml_border, "d", &d);
    lane_border.set_d(d);
    ele_lane.mutable_borders()->emplace_back(lane_border);
    curr_xml_border = XmlNextSiblingElement(curr_xml_border);
  }
  common::VectorSortPoloy3(ele_lane.mutable_borders());
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneRoadMarkEle(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  const tinyxml2::XMLElement* curr_xml_mark =
      xml_lane->FirstChildElement("roadMark");
  double s;
  double width;
  double height;
  while (curr_xml_mark) {
    element::RoadMark road_mark;
    s = road_mark.s();
    width = road_mark.width();
    height = road_mark.height();
    XmlQueryDoubleAttribute(curr_xml_mark, "sOffset", &s);
    road_mark.set_s(s);
    XmlQueryDoubleAttribute(curr_xml_mark, "width", &width);
    road_mark.set_width(width);
    XmlQueryDoubleAttribute(curr_xml_mark, "height", &height);
    road_mark.set_height(height);
    XmlQueryStringAttribute(curr_xml_mark, "material",
                            road_mark.mutable_material());
    // common::XmlQueryEnumAttribute(
    //     curr_xml_mark, "type", road_mark.mutable_type(),
    //     ROADMARK_TYPE_CHOICES);
    // common::XmlQueryEnumAttribute(curr_xml_mark, "color",
    //                               road_mark.mutable_color(),
    //                               ROAD_MARK_COLOR_CHOICES);
    // common::XmlQueryEnumAttribute(curr_xml_mark, "weight",
    //                               road_mark.mutable_weight(),
    //                               ROAD_MARK_WEIGHT_CHOICES);
    // common::XmlQueryEnumAttribute(curr_xml_mark, "laneChange",
    //                               road_mark.mutable_lane_change(),
    //                               ROAD_MARK_LANE_CHANGE_CHOICES);
    ele_lane.mutable_road_marks()->emplace_back(road_mark);
    curr_xml_mark = XmlNextSiblingElement(curr_xml_mark);
  }
  common::VectorSortPoloy3(ele_lane.mutable_road_marks());
  return *this;
}

SectionXmlParser& SectionXmlParser::ParseLaneSpeedEle(
    const tinyxml2::XMLElement* xml_lane, element::Lane& ele_lane) {
  if (!ok()) return *this;
  const tinyxml2::XMLElement* curr_xml_speed =
      xml_lane->FirstChildElement("speed");
  double s;
  float speed_max;
  while (curr_xml_speed) {
    element::LaneSpeed lane_speed;
    s = lane_speed.s();
    speed_max = lane_speed.max();
    XmlQueryDoubleAttribute(curr_xml_speed, "sOffset", &s);
    lane_speed.set_s(s);
    XmlQueryFloatAttribute(curr_xml_speed, "max", &speed_max);
    lane_speed.set_max(speed_max);
    // common::XmlQueryEnumAttribute(curr_xml_speed, "unit",
    //                               lane_speed.mutable_unit(),
    //                               SPEEDUNIT_CHOICES);
    ele_lane.mutable_max_speeds()->emplace_back(lane_speed);
    curr_xml_speed = XmlNextSiblingElement(curr_xml_speed);
  }
  common::VectorSortPoloy3(ele_lane.mutable_max_speeds());
  return *this;
}

}  // namespace opendrive
}  // namespace hdmap
