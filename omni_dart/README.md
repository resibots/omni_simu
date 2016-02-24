# omni_vrep

#### Here we keep the [DART] integration for our omnidirectional robots.

## Authors

- Author/Maintainer: Federico Allocati

## How to compile

### Dependencies
- [Boost]: C++ Template Libraries
- [Eigen]: C++ Linear Algebra Library
- [DART]: DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open source library that provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation. We use the **upstream version with bullet integration**:
    - Get the code with `git clone https://github.com/dartsim/dart.git`
    - Make sure you have installed all of the dependencies listed [here](https://github.com/dartsim/dart/wiki/DART%205.1%20Installation%20for%20Ubuntu#install-required-dependencies), including Bullet Collision Detector Support.
    - Go to the `dart` folder
    - `mkdir build && cd build`
    - Configure with `cmake ..`. Optionally you can add `-DCMAKE_INSTALL_PREFIX:PATH=/path/to/install` to install it in a custom directory.
    - Compile with `make -j4`
    - Install with `sudo make install`

### Compile and install

- cd to `omni_dart` folder
- Configure with `./waf configure --prefix=/path/to/install`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Copy the file `waf_tools/omni_dart.py` to the folder where your wscript is.

Then add the following line to your `options(opt)` method:

```python
opt.load('omni_dart')
```

And the following lines to your `configure(conf)` method:

```python
conf.load('omni_dart')
conf.check_omni_dart()
```

You can specify an additional parameter `required=True` in the `check_omni_vrep` method, to force the configuration to fail is the library is not found.
Also, if you require mandatort graphics support, you can specify another parameter `require_graphics=True` in the same method.

## LICENSE

[CeCILL]

[DART]: http://dartsim.github.io/
[Boost]: http://www.boost.org
[Eigen]: http://eigen.tuxfamily.org/
[CeCILL]: http://www.cecill.info/index.en.html