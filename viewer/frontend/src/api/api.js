import axios from '@/utils/axios'

const api_all = {
  get_global_map() {
    return axios({
      url: '/opendrive/engine/map/',
      method: 'GET'
    })
  },
  get_nearest_lane(x, y) {
    return axios({
      url: '/opendrive/engine/nearest_lane/',
      method: 'POST',
      data: {
        "x": x,
        "y": y,
      }

    })
  },
};

export default api_all
