# OpenDRIVE Engine

[![Build Status](https://github.com/minhanghuang/opendrive-engine/actions/workflows/ubuntu-build.yaml/badge.svg?branch=dev)](https://github.com/minhanghuang/opendrive-engine/actions/workflows/ubuntu-build.yaml)
[![Build Status](https://github.com/minhanghuang/opendrive-engine/actions/workflows/macos-build.yaml/badge.svg?branch=dev)](https://github.com/minhanghuang/opendrive-engine/actions/workflows/macos-build.yaml)

![opendrive-engine](./docs/images/opendrive-engine.png)

## Dependence

- [opendrive-parse](https://github.com/minhanghuang/opendrive-cpp)
- [tinyxml2](https://github.com/leethomason/tinyxml2)
- [nanoflann](https://github.com/jlblancoc/nanoflann)
- [http-server](https://github.com/minhanghuang/cyclone)

## QuickStart

### install dependence

```bash
python3 setup.py
```

### run server

```bash
./scripts/run_viewer.sh
```

### run frontend

```bash
cd viewer/frontend
npm install
npm run dev
```

## Feature

- [ ] The UI interface displays the information of the current point and shows the current lane lines.

- [ ] Path planning and displaying the planned path.

## References

[imap](https://github.com/daohu527/imap)
