import router from '../router/index'
import axios from 'axios'

axios.defaults.baseURL = "http://" + window.location.hostname +":9070/";

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

export default axios
