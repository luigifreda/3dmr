# An Interface for Message Logging

## Overview

This package provides an interface to log text messages. If the library is used on an installed [ROS system](http://www.ros.org), the backend uses the [ROS logging mechanism](http://wiki.ros.org/roscpp/Overview/Logging). Otherwise the messages are forwarded to [std::cout](http://www.cplusplus.com/reference/iostream/cout/).

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author(s):** Christian Gehring
**Date:** Dec. 2014

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

### Selecting the Backend

This library defines macros to wrap to ROS logger macros or to std::cout calls. The default backend is the ROS logger. You can enable the usage of std::cout by providing defining `MELO_USE_COUT` in the `CMakeLists.txt` of your package:

```
add_definitions(-DMELO_USE_COUT)
```

### Logging Levels

Five logging levels are defined: debug, info, warning, error, fatal.

The following logging levels are compiled per default:

**std::cout built in Debug:** debug, info, warning, error, fatal.

**std::cout built in Release:** info, warning, error, fatal.

**ROS built in Debug or Release:** debug, info, warning, error, fatal.

Note that when using the ROS backend, even though the debug level is kept when compiling, it does not show up in the console.
This can be changed by calling the `set_logger_level` service of your node change the logger level online.

To change change which logger levels are kept when compiling, set the `MELO_MIN_SEVERITY` to your desired level in the `CMakeLists.txt` of your package, e.g.:

```
add_definitions(-DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)
```

The available levels are

* `MELO_SEVERITY_DEBUG`: Keep all logs.
* `MELO_SEVERITY_INFO`:  Remove debug logs.
* `MELO_SEVERITY_WARN`:  Remove debug and info logs.
* `MELO_SEVERITY_ERROR`: Keep only error and fatal logs.
* `MELO_SEVERITY_FATAL`: Keep only fatal logs.
* `MELO_SEVERITY_NONE`:  Remove all logs.

The fatal logging variants throw an exception and therefore terminate the program, if the exception is not captured.

### Additional Output

This library can also prepend the class and function name of the log function callers, e.g. `void MyClass::MyFunction() { MELO_INFO("bla bla"); }` prints `[Info] [Timestamp] [MyClass::MyFunction] bla bla`.
You can enable this feature by defining `MELO_FUNCTION_PRINTS` in the `CMakeLists.txt` of your package:

```
add_definitions(-DMELO_FUNCTION_PRINTS)
```

Note that when using the ROS backend, it is better to use its [built in functionality](http://wiki.ros.org/rosconsole#Console_Output_Formatting) to print more information.
