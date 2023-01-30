import axios from '@/utils/axios' // 导入axios

const api_all = {
    get_info() {
        return axios({
            url: '/hdmap/info/',
            method: 'GET'
        })
    },
    get_walls() {
        return axios({
            url: '/hdmap/walls/',
            method: 'GET'
        })
    },
    nearest_lane(x,y) {
        if (x == null || y == null) {
        } else  {
            return axios({
                url: '/hdmap/nearest/',
                method: 'POST',
                data: {
                    "x":x,
                    "y":y
                }
            })
        }
    },
    path_planning(username, way_points, trunk_project, up, down, loop, dest_type) {
        if (up == null) {
          up = "";
        }
        if (down == null) {
          down = "";
        }
        if (loop == null) {
          loop = "";
        }
        if (way_points == null) {

        } else {
            return axios({
                url: '/hdmap/planning/',
                method: 'POST',
                data: {
                    "username": username,
                    "way_points":way_points,
                    "project":trunk_project,
                    "up":up,
                    "down":down,
                    "loop": loop,
                    "dest_type": dest_type
                }
            })
        }
    },
    local_map(username, x, y) {
        if (x !== 0 && y !== 0) {
            return axios({
                url: '/hdmap/localmap/',
                method: 'POST',
                data: {
                    "username": username,
                    "x":x,
                    "y":y
                }
            })
        }
    },
    clear_areas(id) {
      return axios({
          url: '/hdmap/clearareas/',
          method: 'POST',
          data: {
              "id":id,
          }
      })
    },
    search_objects(x, y, range) { 
        if (x !== 0 && y !== 0) { 
            return axios({
                url: 'hdmap/objects',
                method: 'POST',
                data: {
                    "x": x,
                    "y": y,
                    "range": range
                }
            })
        }
    }
};

export default api_all // 导出 api_all
