<template>
  <div class="layout">
    <Layout>
      <Content :style="{ background: 'rgba(231,231,231,1)' }">
        <div class="left"></div>
        <div id="tool">
          <br />
          <Button type="success">剩余距离</Button>
        </div>
      </Content>
    </Layout>
  </div>
</template>

<script>
import "ol/ol.css";
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
      map: {},
      layers: {},
      view_center: [0, 0],
      mouse_x: 0,
      mouse_y: 0,
    };
  },
  created() {},
  mounted() {
    this.Init();
  },
  methods: {
    Init() {
      this.initOpenLayer();
      this.showGlobalMap();
    },
    initOpenLayer() {
      console.log("initOpenLayer()");
      var mousePositionControl = new MousePosition({
        coordinateFormat: createStringXY(4),
        projection: "EPSG:3857",
        className: "custom-mouse-position",
        target: document.getElementById("mouse-position"),
        undefinedHTML: "&nbsp;",
      });
      this.map = new Map({
        controls: defaultControls().extend([mousePositionControl]),
        layers: [],
        target: "map",
        projection: "EPSG:3857",
        view: new View({
          // https://openlayers.org/en/latest/apidoc/module-ol_View-View.html
          center: this.view_center,
          zoom: 18.5,
          rotation: 0, // 视图的初始旋转以弧度为单位（顺时针正旋转，0 表示北）。
          maxZoom: 40,
          minZoom: 1,
        }),
      }); // map
      const dblClickInteraction = this.map
        .getInteractions()
        .getArray()
        .find((interaction) => {
          return interaction instanceof DoubleClickZoom;
        });
      this.map.removeInteraction(dblClickInteraction); // 双击禁止放大
      this.map.on("pointermove", function () {
        var mouseText = document
          .getElementById("mouse-position")
          .textContent.split(", ");
        this.mouse_x = parseFloat(mouseText[0]);
        this.mouse_y = parseFloat(mouseText[1]);
        this.getCursorData();
      });
      this.map.on("click", function () {});
      this.map.on("singleclick", function () {
        console.log("singleclick");
        this.getNearsetPoint();
      });
      this.map.on("dblclick", function () {
        console.log("dblclick");
      });
      mousePositionControl.setProjection("EPSG:3857");
      mousePositionControl.setCoordinateFormat(createStringXY(4));
    },
    showGlobalMap() {
      console.log("showGlobalMap()");
      this.$api.api_all
        .get_global_map()
        .then((response) => {
          console.log("GlobalMap Data: ", response);
          const data = response.data;
          if (data.code != 1000) {
            console.log("response exec: ", data.code);
            return;
          }
        })
        .catch((error) => {});
    }, // showGlobalMap() end
    showLine(layer_name, data, color, z, remove = true, type = "LineString") {
      if (remove && this.layers[layer_name]) {
        this.map.removeLayer(this.layers[layer_name]);
      }

    }, // showLine() end
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
