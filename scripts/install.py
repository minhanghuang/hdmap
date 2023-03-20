import os
import sys


class Worker:

    def __init__(self) -> None:
        self.current_path = os.path.abspath(__file__)
        self.project_path = os.path.dirname(
            os.path.dirname(self.current_path))
        self.download_path = os.path.join(self.project_path, "third_party")
        self.install_path = os.path.join(self.project_path, "install")
        self.install_list = {
            "tinyxml2": "https://github.com/leethomason/tinyxml2.git",
            "setup": "https://github.com/minhanghuang/setup.git",
            "cactus": "https://github.com/minhanghuang/cactus.git",
            "cyclone": "https://github.com/minhanghuang/cyclone.git",
            # "opendrive-cpp":"https://github.com/minhanghuang/opendrive-cpp.git",
            "opendrive-cpp": "git@github.com:minhanghuang/opendrive-cpp.git",
            "yaml-cpp": "https://github.com/jbeder/yaml-cpp.git",
            "nanoflann": "https://github.com/jlblancoc/nanoflann.git",
            "googletest": "https://github.com/google/googletest.git",
        }
        self.xodr_list = [
            "https://github.com/minhanghuang/opendrive-files/blob/main/carla-simulator/Town07.xodr",
        ]

    def download(self):
        print("download...")
        print("current_path: {}".format(self.current_path))
        print("download_path: {}".format(self.download_path))
        print("install_path: {}".format(self.install_path))
        for pro, repo in self.install_list.items():
            download_path = os.path.join(self.download_path, pro)
            if os.path.exists(download_path):
                print("{} exists.".format(download_path))
                continue
            cmd = "git clone --depth 1 {} {}".format(repo, download_path)
            print("cmd: {}".format(cmd))
            os.system(cmd)
        self.__install_xodr_file()

    def install(self):
        print("install...")
        for pro in os.listdir(self.download_path):
            path = os.path.join(self.download_path, pro)
            print(path)
            cmd = "export PKG_CONFIG_PATH={0}/lib/pkgconfig:{0}/share/pkgconfig:$PKG_CONFIG_PATH && cd {1} && cmake -B build -DCMAKE_INSTALL_PREFIX={0} -DBUILD_SHARED_LIBS=ON && cmake --build build -j4 && cd build && make install".format(
                self.install_path, path)
            os.system(cmd)

    def __install_xodr_file(self):
        for path in self.xodr_list:
            cmd = "wget -nc {} -P {}".format(path,
                                             os.path.join(self.install_path, "share/xodr/"))
            print("cmd: {}".format(cmd))
            os.system(cmd)


def main():
    worker = Worker()
    worker.download()
    worker.install()


if __name__ == "__main__":
    main()
