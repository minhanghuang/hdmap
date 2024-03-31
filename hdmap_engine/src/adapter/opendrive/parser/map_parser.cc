#include "hdmap_engine/adapter/opendrive/parser/map_parser.h"

namespace hdmap {
namespace opendrive {

MapXmlParser::MapXmlParser() {}

bool MapXmlParser::Parse(const tinyxml2::XMLElement* xml_map,
                         element::Map* ele_map) {
  xml_map_ = xml_map;
  ele_map_ = ele_map;
  if (!xml_map_ || !ele_map_) {
    UpdateStatus(ErrorCode::XML_ROAD_ELEMENT_ERROR, "Input is null.");
    return false;
  }

  ParseJunctionElement().ParseRoadElement();
  return true;
}

MapXmlParser& MapXmlParser::ParseJunctionElement() {
  if (!ok()) return *this;
  // 0~*
  const tinyxml2::XMLElement* curr_xml_junction =
      xml_map_->FirstChildElement("junction");
  while (curr_xml_junction) {
    element::Junction ele_junction;
    int id = ele_junction.mutable_attribute()->id();
    int main_road = ele_junction.mutable_attribute()->main_road();
    double s = ele_junction.mutable_attribute()->start_position();
    double e = ele_junction.mutable_attribute()->end_position();
    XmlQueryIntAttribute(curr_xml_junction, "id", &id);
    ele_junction.mutable_attribute()->set_id(id);
    XmlQueryIntAttribute(curr_xml_junction, "mainRoad", &main_road);
    ele_junction.mutable_attribute()->set_main_road(main_road);
    XmlQueryDoubleAttribute(curr_xml_junction, "sStart", &s);
    ele_junction.mutable_attribute()->set_start_position(s);
    XmlQueryDoubleAttribute(curr_xml_junction, "sEnd", &e);
    ele_junction.mutable_attribute()->set_end_position(e);
    XmlQueryStringAttribute(curr_xml_junction, "name",
                            ele_junction.mutable_attribute()->mutable_name());
    // common::XmlQueryEnumAttribute(
    //     curr_xml_junction, "orientation",
    //     ele_junction.mutable_attribute()->mutable_dir(), DIR_CHOICES);
    // common::XmlQueryEnumAttribute(
    //     curr_xml_junction, "type",
    //     ele_junction.mutable_attribute()->mutable_type(),
    //     JUNCTION_TYPE_CHOICES);
    // junction connection
    // 1~*
    const tinyxml2::XMLElement* curr_xml_connection =
        curr_xml_junction->FirstChildElement("connection");
    if (!curr_xml_connection) {
      UpdateStatus(ErrorCode::XML_JUNCTION_CONNECTION_ELEMENT_ERROR,
                   "JUNCTION CONNECTION ELEMENT IS NULL.");
      return *this;
    }
    while (curr_xml_connection) {
      element::JunctionConnection connection;
      int connection_id = connection.id();
      int linked_road = connection.linked_road();
      int incoming_road = connection.incoming_road();
      int connecting_road = connection.connecting_road();
      XmlQueryIntAttribute(curr_xml_connection, "id", &connection_id);
      connection.set_id(connection_id);
      // common::XmlQueryEnumAttribute(curr_xml_connection, "type",
      //                               connection.mutable_type(),
      //                               JUNCTION_CONNECTION_TYPE_CHOICES);
      XmlQueryIntAttribute(curr_xml_connection, "linkedRoad", &linked_road);
      connection.set_linked_road(linked_road);
      XmlQueryIntAttribute(curr_xml_connection, "incomingRoad", &incoming_road);
      connection.set_incoming_road(incoming_road);
      XmlQueryIntAttribute(curr_xml_connection, "connectingRoad",
                           &connecting_road);
      connection.set_connecting_road(connecting_road);
      // common::XmlQueryEnumAttribute(curr_xml_connection, "contactPoint",
      //                               connection.mutable_contact_point(),
      //                               CONTACT_POINT_TYPE_CHOICES);
      // connection link
      // 0~*
      const tinyxml2::XMLElement* curr_xml_laneLink =
          curr_xml_connection->FirstChildElement("laneLink");
      while (curr_xml_laneLink) {
        element::JunctionLaneLink lane_link;
        int from = lane_link.from();
        int to = lane_link.to();
        XmlQueryIntAttribute(curr_xml_laneLink, "from", &from);
        lane_link.set_from(from);
        XmlQueryIntAttribute(curr_xml_laneLink, "to", &to);
        lane_link.set_to(to);
        connection.mutable_lane_links()->emplace_back(lane_link);
        curr_xml_laneLink = XmlNextSiblingElement(curr_xml_laneLink);
      }
      ele_junction.mutable_connections()->emplace_back(connection);
      curr_xml_connection = XmlNextSiblingElement(curr_xml_connection);
    }
    ele_map_->mutable_junctions()->emplace_back(ele_junction);
    curr_xml_junction = XmlNextSiblingElement(curr_xml_junction);
  }
  return *this;
}

MapXmlParser& MapXmlParser::ParseRoadElement() {
  if (!ok()) return *this;
  // 1~*
  const tinyxml2::XMLElement* curr_xml_road =
      xml_map_->FirstChildElement("road");
  if (!curr_xml_road) {
    UpdateStatus(ErrorCode::XML_ROAD_ELEMENT_ERROR, "ROAD ELEMENT IS NULL.");
    return *this;
  }
  while (curr_xml_road) {
    element::Road ele_road;
    RoadXmlParser::Parse(curr_xml_road, &ele_road);
    ele_map_->mutable_roads()->emplace_back(ele_road);
    curr_xml_road = XmlNextSiblingElement(curr_xml_road);
  }
  return *this;
}

}  // namespace opendrive
}  // namespace hdmap
