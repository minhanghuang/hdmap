<template>
  <div class="layout">
    <Layout>
      <Content :style="{ background: 'rgba(231,231,231,1)' }">
        <div class="left">
          <div id="map">
            <div id="mouse-position"></div>
          </div>
        </div>
        <div id="tool">
          <br />
          <Button type="success" @click="clickClear">Clear</Button>
        </div>
      </Content>
    </Layout>
  </div>
</template>

<script>
import "ol/ol.css";
import GUI from "lil-gui";
import Map from "ol/Map";
import View from "ol/View";
import { Vector as VectorLayer } from "ol/layer";
import { Vector as VectorSource } from "ol/source";
import { Fill } from "ol/style";
import { Stroke, Style, Icon, Circle, Text } from "ol/style";
import GeoJSON from "ol/format/GeoJSON";
import Collection from "ol/Collection";
import { defaults as defaultControls } from "ol/control";
import MousePosition from "ol/control/MousePosition";
import { createStringXY } from "ol/coordinate";
import { DoubleClickZoom } from "ol/interaction";
import { Feature } from "ol";
import { Point } from "ol/geom";
import { Polygon } from "ol/geom";
import { Cluster } from "ol/source";
export default {
  name: "Home",
  data() {
    return {
      username: "xodr-engine",
      base_map_layer_name: "global_map",
      gui: {},
      map: {},
      styles: {},
      layers: {},
      point_layers: {},
      view_center: [0, 0],
      mouse: [0, 0], // 鼠标
      debounce_timer: null, // 鼠标防抖
      singleclick_mouse: [0, 0], // 单击鼠标
      way_points: [], // 途经点
    };
  },
  created() {},
  mounted() {
    this.init();
  },
  methods: {
    init() {
      var self = this;
      self.initOl();
      self.initOlStyle();
      self.initGui();
      self.initWs();
      self.showGlobalMap();
    },
    initOl() {
      console.log("initOpenLayer()");
      var self = this;
      var mousePositionControl = new MousePosition({
        coordinateFormat: createStringXY(4),
        projection: "EPSG:3857",
        className: "custom-mouse-position",
        target: document.getElementById("mouse-position"),
        undefinedHTML: "&nbsp;",
      });
      self.map = new Map({
        controls: defaultControls().extend([mousePositionControl]),
        layers: [],
        target: "map",
        projection: "EPSG:3857",
        view: new View({
          // https://openlayers.org/en/latest/apidoc/module-ol_View-View.html
          center: self.view_center,
          zoom: 18.5,
          rotation: 0, // 视图的初始旋转以弧度为单位（顺时针正旋转，0 表示北）。
          maxZoom: 40,
          minZoom: 1,
        }),
      }); // map
      const dblClickInteraction = self.map
        .getInteractions()
        .getArray()
        .find((interaction) => {
          return interaction instanceof DoubleClickZoom;
        });
      self.map.removeInteraction(dblClickInteraction); // 双击禁止放大
      self.map.on("pointermove", function () {
        var mouseText = document
          .getElementById("mouse-position")
          .textContent.split(", ");
        self.mouse[0] = parseFloat(mouseText[0]);
        self.mouse[1] = parseFloat(mouseText[1]);
        self.clickCursor();
      });
      self.map.on("click", function () {});
      self.map.on("singleclick", function () {
        console.log("singleclick");
        var mouseText = document
          .getElementById("mouse-position")
          .textContent.split(", ");
        self.singleclick_mouse[0] = parseFloat(mouseText[0]);
        self.singleclick_mouse[1] = parseFloat(mouseText[1]);
        self.showNearestLane(); // 显示最近车道
        self.showWayPoint("way_points", self.singleclick_mouse, 10, false); // 显示最近点
        self.saveWayPoint(self.singleclick_mouse);
      });
      self.map.on("dblclick", function () {
        console.log("dblclick");
      });
      mousePositionControl.setProjection("EPSG:3857");
      mousePositionControl.setCoordinateFormat(createStringXY(4));
    },
    initOlStyle() {
      console.log("initOlStyle()");
      var self = this;
      var lineStringGlobalMapStyle = new Style({
        stroke: new Stroke({
          color: "rgba(90, 200, 254, 0.9)",
          width: 2,
        }),
      });
      var lineStringNearestLaneStyle = new Style({
        stroke: new Stroke({
          color: "#DB7093",
          width: 2,
        }),
      });

      self.styles["lineStringGlobalMapStyle"] = lineStringGlobalMapStyle;
      self.styles["lineStringNearestLaneStyle"] = lineStringNearestLaneStyle;
    }, // initOlStyle() end
    initGui() {
      console.log("initGui()");
      var self = this;
      self.gui = new GUI();
      self.gui.add(document, "title");
    },
    initWs() {
      var self = this;
      const url =
        "ws://" +
        window.location.hostname +
        ":9070/opendrive/engine/ws/real_time_data/";
      console.log("ws url: ", url);
      self.$websocket.createSocket(url);
      const onMessage = (e) => {
        // 创建接收消息函数
        const data_str = e && e.detail.data;
        console.log(data_str);
        if ("" == data_str) {
          return;
        }
        let data = JSON.parse(data_str);
      };
      window.addEventListener("onmessageWS", onMessage);
    },

    clearLayers() {
      console.log("clearLayers()");
      var self = this;
      // line layer
      var keys = Object.keys(self.layers);
      for (let i = 0; i < keys.length; i++) {
        if (keys[i] != self.base_map_layer_name) {
          self.map.removeLayer(self.layers[keys[i]]);
          delete self.layers[keys[i]];
        }
      }
      // point layer
      var keys = Object.keys(self.point_layers);
      for (let i = 0; i < keys.length; i++) {
        self.map.removeLayer(self.point_layers[keys[i]]);
        delete self.point_layers[keys[i]];
      }
    }, // clearLayers() end

    clickCursor() {
      var self = this;
      if (self.debounce_timer) {
        clearTimeout(self.debounce_timer);
      }
      self.debounce_timer = setTimeout(() => {
        // 这里写入需要执行的代码
        self.$websocket.sendWSPush({ x: self.mouse[0], y: self.mouse[1] });
        console.log("Mouse move debounced");
      }, 500); // 防抖间隔为500毫秒
    }, // clickCursor() end

    /*   ----              -----  */
    /*   ----  show layer  -----  */
    /*   ----              -----  */
    showGlobalMap() {
      console.log("showGlobalMap()");
      var self = this;
      self.$api.api_all
        .get_global_map()
        .then((response) => {
          const code = response.data.code;
          const data = response.data.results;
          if (code != 1000) {
            console.log("response exec: ", code);
            return;
          }
          var lines = [];
          for (var i = 0; i < data.length; i++) {
            var parse_lane = self.parseLane(data[i]);
            lines.push(parse_lane["left_boundary"]);
            lines.push(parse_lane["right_boundary"]);
          }
          self.view_center = lines[0][0][0];
          self.showLines(
            self.base_map_layer_name,
            lines,
            "lineStringGlobalMapStyle",
            1,
            true
          );
        })
        .catch((error) => {});
    }, // showGlobalMap() end

    showNearestLane() {
      console.log("showNearestLane()");
      var self = this;
      self.$api.api_all
        .get_nearest_lane(self.singleclick_mouse[0], self.singleclick_mouse[1])
        .then((response) => {
          console.log("NearestLane Data: ", response);
          const data = response.data;
          if (data.code != 1000) {
            console.log("response exec: ", data.code);
            return;
          }
          var lines = [];
          var parse_lane = self.parseLane(data.results);
          lines.push(parse_lane["left_boundary"]);
          lines.push(parse_lane["right_boundary"]);
          self.showLines(
            "nearest_lane",
            lines,
            "lineStringNearestLaneStyle",
            1,
            true
          );
        })
        .catch((error) => {});
    }, // showNearestLane() end

    showLines(
      layer_name,
      data /* lines [[[1,2], [3,4]], [[2,4], [5,8]]]*/,
      style,
      z,
      remove = true
    ) {
      var self = this;
      if (!self.styles[style]) {
        console.log("style不存在");
        return;
      }
      if (remove && self.layers[layer_name]) {
        self.map.removeLayer(self.layers[layer_name]);
      }
      var source = {
        type: "FeatureCollection",
        features: [],
      };
      for (var i = 0; i < data.length; i++) {
        source["features"].push({
          type: "Feature",
          geometry: { type: "LineString", coordinates: data[i] },
        });
      }
      var vectorSource = new VectorSource({
        features: new GeoJSON().readFeatures(source),
      });
      var vectorLayer = new VectorLayer({
        source: vectorSource,
        style: self.styles[style],
        zIndex: z,
      });
      self.map.addLayer(vectorLayer);
      self.layers[layer_name] = vectorLayer;
    }, // showLine() end

    showWayPoint(layer_name, data /*[2,4]*/, z, remove = true) {
      var self = this;
      if (remove || self.point_layers[layer_name]) {
        self.map.removeLayer(self.point_layers[layer_name]);
      }

      var features = new Array();
      features.push(new Feature({ geometry: new Point(data) }));
      var source = new VectorSource({
        features: features,
      });
      var clusterSource = new Cluster({
        distance: 0,
        source: source,
      });
      var style = new Style({
        image: new Circle({
          radius: 7,
          stroke: new Stroke({
            color: "red",
          }),
          fill: new Fill({
            color: "red",
          }),
        }),
        text: new Text({
          font: "13px Calibri,sans-serif",
          fill: new Fill({ color: "#000" }),
          stroke: new Stroke({
            color: "#fff",
            width: 2,
          }),
          text: Object.keys(self.point_layers).length.toString(),
        }),
      });

      var vector_layer = new VectorLayer({
        source: clusterSource,
        style: style,
        zIndex: 10,
      });

      self.map.addLayer(vector_layer);
      self.point_layers[Object.keys(self.point_layers).length.toString()] =
        vector_layer;
    }, // showWayPoint() end

    /*   ----                  -----  */
    /*   ----     function     -----  */
    /*   ----                  -----  */
    saveWayPoint(point) {
      var self = this;
      self.way_points.push(Array.from(point));
    }, // saveWayPoint() end
    // 解析车道
    parseLane(lane) {
      var left_boundary = [];
      var right_boundary = [];
      if (lane.left_boundary.length == lane.right_boundary.length) {
        for (var i = 0; i < lane.left_boundary.length; i++) {
          left_boundary.push([
            lane.left_boundary[i].x,
            lane.left_boundary[i].y,
          ]);
          right_boundary.push([
            lane.right_boundary[i].x,
            lane.right_boundary[i].y,
          ]);
        }
      }
      return { left_boundary: left_boundary, right_boundary: right_boundary };
    }, // parseLane() end
    /*   ----                  -----  */
    /*   ----  click callback  -----  */
    /*   ----                  -----  */
    clickClear() {
      var self = this;
      self.clearLayers();
    }, // clickClear() end
  },
};
</script>

<style scoped>
.layout {
  border: 1px solid #d7dde4;
  background: #f5f7f9;
  position: relative;
  border-radius: 4px;
  overflow: hidden;
  height: 100%;
  width: 100%;
}

.ivu-layout {
  height: 100%;
}

.left {
  box-shadow: 0px -4px 4px 0px rgba(140, 146, 165, 0.3);
  float: right;
  width: 85%;
  height: 100%;
  overflow: hidden;
  background: rgba(245, 246, 247, 1);
}

#map {
  height: 100%;
}

#mouse-position {
  position: absolute;
  bottom: 80px;
}

#tool {
  float: right;
  width: 15%;
  height: 100%;
  padding: 10px;
  background: rgba(245, 246, 247, 1);
  box-shadow: 0px 2px 7px 0px rgba(174, 166, 166, 0.5);
  border-radius: 4px;
}
</style>
