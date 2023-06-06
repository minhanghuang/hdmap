import sys
import os

DEPENDENCES = {
    # "": [link, branch, cmake options]
    "setup": ["https://github.com/minhanghuang/setup.git", "", ""],
    "googletest": ["https://github.com/google/googletest.git", "", "-DCMAKE_CXX_STANDARD=14"],
    "tinyxml2": ["https://github.com/leethomason/tinyxml2.git", "", ""],
    "cactus": ["https://github.com/minhanghuang/cactus.git", "", ""],
    "cyclone": ["https://github.com/minhanghuang/cyclone.git", "", ""],
    "opendrive-cpp": ["https://github.com/minhanghuang/opendrive-cpp.git", "", ""],
    "yaml-cpp": ["https://github.com/jbeder/yaml-cpp.git", "", ""],
    "nanoflann": ["https://github.com/jlblancoc/nanoflann.git", "", ""],
    "json": ["https://github.com/nlohmann/json.git", "", ""],
    "opendrive-files": ["https://github.com/minhanghuang/opendrive-files.git", "", ""],
}


class Repository:
    def __init__(self) -> None:
        self.__link = ""
        self.__name = ""
        self.__branch = ""

    def get_link(self):
        return self.__link

    def get_name(self):
        return self.__name

    def get_branch(self):
        return self.__branch

    def set_link(self, link):
        self.__link = link

    def set_name(self, name):
        self.__name = name

    def set_branch(self, branch):
        self.__branch = branch


class Pipeline:
    def __init__(self) -> None:
        self.__repos = []
        self.__current_path = os.path.abspath(__file__)
        self.__download_path = os.path.join(
            os.path.dirname(self.__current_path), "third_party")
        self.__install_path = os.path.join(
            os.path.dirname(self.__current_path), "install")

    def init(self):
        print("current path: {}".format(self.__current_path))
        print("download path: {}".format(self.__download_path))
        print("install path: {}".format(self.__install_path))
        os.system("mkdir -p {}".format(self.__download_path))
        os.system("mkdir -p {}".format(self.__install_path))

    def append_repository(self, link: str, branch: str = ""):
        repo = Repository()
        repo.set_link(link=link)
        repo.set_name(name=link.rsplit(".", 1)[0].rsplit("/", 1)[-1])
        repo.set_branch(branch=branch)
        self.__repos.append(repo)

    def download(self):
        print("########### Download...")
        for repo in self.__repos:
            self.__clone(repo=repo)

    def build(self):
        print("########### Build...")
        for name in os.listdir(self.__download_path):
            print("name: ", name)
            os.chdir(os.path.join(self.__download_path, name))
            cmd = "mkdir -p build && cd build && cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX={} {}".format(
                self.__install_path, DEPENDENCES[name][2])
            print("cmake: {}".format(cmd))
            os.system(cmd)
            os.chdir("build")
            cmd = "make install -j4"
            print("make: {}".format(cmd))
            os.system(cmd)

    def upload(self):
        print("########### Upload...")
        cmd = "mkdir -p {} && cp -r {} {}".format("/opt/xodr", os.path.join(self.__install_path,
                                                                            "share/"), "/opt/xodr/")
        print("{}".format(cmd))
        os.system(cmd)

    def __clone(self, repo: Repository):
        cmd = ""
        download_path = os.path.join(self.__download_path, repo.get_name())
        if os.path.exists(download_path):
            return
            # os.system("rm -rf {}".format(download_path))
        os.system("mkdir -p {}".format(download_path))
        if "" == repo.get_branch():
            cmd = "git clone --depth 1 {} {}".format(
                repo.get_link(),
                download_path)
        else:
            cmd = "git clone --depth 1 --single-branch --branch {} {} {}".format(
                repo.get_branch(),
                repo.get_link(),
                download_path)

        print("clone: {}".format(cmd))
        os.system(cmd)


def main():
    pipe_line = Pipeline()
    pipe_line.init()
    for _, repo in DEPENDENCES.items():
        print("parse: {} {}".format(repo[0], repo[1]))
        pipe_line.append_repository(link=repo[0], branch=repo[1])
    pipe_line.download()
    pipe_line.build()
    pipe_line.upload()


if __name__ == "__main__":
    main()
