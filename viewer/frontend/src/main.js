// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
import Vue from 'vue'
import App from './App'
import router from './router'
import axios from "./api/"; // 导入axios
import ViewUI from 'view-design';
import 'view-design/dist/styles/iview.css';
import websocket from './utils/websocket.js'

Vue.use(ViewUI);
Vue.prototype.$api = axios; // 挂载到原型链上, 在vue组件里面可以通过this拿到
Vue.prototype.$websocket = websocket; // 挂载
Vue.config.productionTip = false;

new Vue({
    el: '#app',
    router,
    // store,
    components: { App },
    template: '<App/>'
});

