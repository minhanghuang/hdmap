import os
import sys


class Worker:

    def __init__(self) -> None:
        self.current_path = os.path.abspath(__file__)
        self.project_path = os.path.dirname(
            os.path.dirname(self.current_path))

    def run(self):
        cmd = "cd {} && cmake -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_OPENDRIVE_ENGINE_VIEWER=ON . && cmake --build build -j6".format(
            self.project_path)
        print("cmd: {}".format(cmd))
        os.system(cmd)
        cmd = "cd {} && build/viewer/backend/engine_server_runner build/conf/engine_server.yaml".format(
            self.project_path)
        print("cmd: {}".format(cmd))
        os.system(cmd)


def main():
    worker = Worker()


if __name__ == "__main__":
    main()
