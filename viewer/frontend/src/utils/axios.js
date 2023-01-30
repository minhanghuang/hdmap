import qs from 'qs'
import router from '../router/index'
import axios from 'axios' // 导入node_modules里的axios


// axios.defaults.baseURL = "http://127.0.0.1:20401/"; // 后端接口 ip:port
// axios.defaults.baseURL = "http://192.168.96.3:20401/"; // 后端接口 ip:port
axios.defaults.baseURL = "http://" + window.location.hostname +":9900/";
// axios.defaults.baseURL = "http://" + "192.168.96.3" +":20401/";
// axios.defaults.baseURL = "http://192.168.96.2:20401/"; // 后端接口 ip:port
// 设置 post、put 默认 Content-Type
// axios.defaults.headers.post["Content-Type"] = "application/json;charset=utf-8";
// axios.defaults.headers.put["Content-Type"] = "application/json;charset=utf-8";


axios.interceptors.request.use((request) => {
    return request;
},(error) =>{
    console.log('错误的传参');
    return Promise.reject(error);
});

// 响应拦截器
axios.interceptors.response.use(function (response) {
    return response
}, function (error) {
    return Promise.reject(error)
});

export default axios // 导出axios

