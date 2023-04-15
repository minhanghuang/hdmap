import os
import sys


current_path = os.path.abspath(__file__)
project_path = os.path.dirname(current_path)
download_path = os.path.join(project_path, "third_party")
install_path = os.path.join(project_path, "install")
xodr_file_list = [
    "https://github.com/minhanghuang/opendrive-files/blob/main/carla-simulator/Town07.xodr",
]
third_party_list = [
    {
        "addr": "https://github.com/leethomason/tinyxml2.git",
        "commit": "master",  # tag branch commit
    },
    {
        "addr": "https://github.com/minhanghuang/setup.git",
    },
    {
        "addr": "https://github.com/minhanghuang/cactus.git",
    },
    {
        "addr": "https://github.com/minhanghuang/cyclone.git",
    },
    {
        "addr": "https://github.com/minhanghuang/opendrive-cpp.git",
    },
    {
        "addr": "https://github.com/jbeder/yaml-cpp.git",
    },
    {
        "addr": "https://github.com/jlblancoc/nanoflann.git",
    },
    {
        "addr": "https://github.com/google/googletest.git",
    },
]


class Plugin:

    def __init__(self) -> None:
        self.__name = ""
        self.__addr = ""
        self.__b = ""

    def get_name(self):
        return self.__name

    def get_addr(self):
        return self.__addr

    def get_b(self):
        return self.__b

    def parse(self, obj: dict):
        if "addr" not in obj.keys():
            return None
        self.__addr = obj["addr"]
        self.__name = self.__addr.rsplit("/")[-1].rsplit(".")[0]
        if "commit" in obj.keys():
            self.__b = obj["commit"]
        return None


class Worker:

    def __init__(self) -> None:
        self.__plugins = []
        self.__parse_plugin()

    def download(self):
        print("download...")
        for item in self.__plugins:
            print("-----", item.get_name())
            print("-----", item.get_addr())
            print("-----", item.get_b())
            dir = os.path.join(download_path, item.get_name())
            if item.get_b():
                cmd = "git clone {} {}".format(
                    item.get_addr(), dir)
                self.command(cmd)
                os.chdir(dir)
                cmd = "git checkout {}".format(item.get_b())
                self.command(cmd)
            else:
                cmd = "git clone --depth 1 {} {}".format(
                    item.get_addr(), dir)
                self.command(cmd)

    def install(self):
        print("install...")
        for project in os.listdir(download_path):
            path = os.path.join(download_path, project)
            cmd = """export PKG_CONFIG_PATH={0}/lib/pkgconfig:{0}/share/pkgconfig:$PKG_CONFIG_PATH && cd {1} &&
                     cmake -B build -DCMAKE_INSTALL_PREFIX={0} -DBUILD_SHARED_LIBS=ON &&
                     cmake --build build -j4 &&
                     cd build &&
                     make install""".format(
                install_path, path)
            self.command(cmd)
        self.__install_xodr_file()

    def command(self, msg):
        print("###cmd: ", msg)
        os.system(msg)

    def __parse_plugin(self):
        for item in third_party_list:
            plugin = Plugin()
            plugin.parse(item)
            self.__plugins.append(plugin)

    def __install_xodr_file(self):
        for path in xodr_file_list:
            cmd = "wget -nc {} -P {}".format(path,
                                             os.path.join(install_path, "share/xodr/"))
            self.command(cmd)


def main():
    worker = Worker()
    worker.download()
    worker.install()


if __name__ == "__main__":
    main()
