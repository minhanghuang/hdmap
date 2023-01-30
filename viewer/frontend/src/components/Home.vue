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
export default {
  name: "Home",
  data() {
    return {
      username: "trunk",
    };
  },
  created() {},
  mounted() {
    this.Init();
  },
  methods: {
    Init() {},
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
