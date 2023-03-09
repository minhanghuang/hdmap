import axios from '@/utils/axios'

const api_all = {
  get_global_map() {
    return axios({
      url: '/opendrive/engine/map/',
      method: 'GET'
    })
  },
};

export default api_all
