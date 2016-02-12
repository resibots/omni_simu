# omni_vrep

#### Here we keep the [v-rep] integration for our omnidirectional robots.

## Authors

- Author/Maintainer: Federico Allocati

## How to compile

### Dependencies
- [Boost]: C++ Template Libraries
- [Eigen]: C++ Linear Algebra Library
- [ROS]: The Robot Operating System
- [V-REP ROS Plugin]: V-REP plugin for integration with ROS

### Compile and install

- cd to `omni_vrep` folder
- Configure with `./waf configure --prefix=path_to_install`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Copy the file `waf_tools/omni_vrep.py` to the folder where your wscript is.

Then add the following line to your `options(opt)` method:

```python
opt.load('omni_vrep')
```

And the following lines to your `configure(conf)` method:

```python
conf.load('omni_vrep')
conf.check_omni_vrep()
```

You can specify an additional parameter `required=True` in the `check_omni_vrep` method, to force the configuration to fail is the library is not found.

## LICENSE

[CeCILL]

[v-rep]: http://www.coppeliarobotics.com/
[Boost]: http://www.boost.org
[ROS]: http://www.ros.org/
[V-REP ROS Plugin]: http://www.coppeliarobotics.com/helpFiles/en/rosTutorialIndigo.htm
[Eigen]: http://eigen.tuxfamily.org/
[CeCILL]: http://www.cecill.info/index.en.html