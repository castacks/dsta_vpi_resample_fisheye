
## Dependencies

This package additionally depends on the [backward-cpp](backward-cpp-github) library. To install `backward-cpp` system-wide

```bash
git clone https://github.com/bombela/backward-cpp.git
cd bacward-cpp
mkdir build
cd build
cmake ..
make
sudo make install
```

Then install the third-party library

```bash
sudo apt-get install libdw-dev
```

[backward-cpp-github]: https://github.com/bombela/backward-cpp