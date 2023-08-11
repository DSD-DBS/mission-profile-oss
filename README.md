<!--
 ~ SPDX-FileCopyrightText: Copyright DB Netz AG
 ~ SPDX-License-Identifier: CC0-1.0
 -->

# Mission Profile

Mission Profile Publisher is a software component which provides the expected journey of a vehicle providing the track identifiers in form of a ROS Message. This component is provided as a submodule in the Rail horzion repository and can only be build together with Rail Horizon.

## Build

### Cloning and Submodules

Mission Profile Publisher will be initialized as a submodule in Rail Horizon, as described in https://github.com/DSD-DBS/rail-horizon-dev/blob/master/README.md using the following command:
```bash
git submodule update --init --recursive
```

### Dependencies

Dependencies (Recommended: Ubuntu 22.04):
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* Python: `sudo apt install python`
* Boost: `sudo apt install libboost-all-dev`

You should also create the underlay, which is the underlying workspace with ROS2 Tools available. It behaves similarly to a virtual environment and allows having multiple ROS2 versions installed on the system and use a specific one for each project. The underlay can be created automatically for every terminal session or manually at the beginning of a new session like this:
```bash
source /opt/ros/humble/setup.bash
```

Check if all dependencies are available with:
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

### Build

Use colcon build to build the project. Mission Profile should be build in the same directory after Rail Horizon is build (described in https://github.com/DSD-DBS/rail-horizon-dev/blob/master/README.md).

```bash
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers console_cohesion+ --packages-up-to dsd_mission_profile
```
- `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` is optional. It generates a compilation database for tooling. For instance, clang-tidy needs it, which is run as a pre-commit.
- `-G Ninja` can optionally be added to the `--cmake-args`. If the `ninja` build system is installed on the system, it can be used instead of make. Builds usually are a lot faster then.

After the build, you should source the install folder:
```bash
source ./install/local_setup.bash
```

### Run

Currently, Mission Profile Publisher contains two configurations for journeys between Berliner Tor - Bergedorf and Bergedorf - Berliner Tor that are given in the parameters1.yaml and parameter2.yaml configuration files, respectively. Mission Profile can be run with the config file specified on the command line.

```bash
ros2 run dsd_mission_profile dsd_mission_profile --ros-args --params-file ./install/dsd_mission_profile/config/dsd_mission_profile/parameters1.yaml
```

### Test

Run tests:
```bash
colcon test --packages-select dsd_mission_profile --return-code-on-test-failure

```
In the build directory of dsd_mission_profile, testing report is created in a Testing directory with a timestamp containing an .xml file.

#### Coverage

Generate coverage report with:
```bash
gcovr --exclude-unreachable-branches --exclude-throw-branches -r . --filter src/ --exclude '.*tests/' --html --html-details -o build/coverage.html
```

The generated file `build/coverage.html` contains the report.

## Licenses

This project is compliant with the [REUSE Specification Version 3.0](https://git.fsfe.org/reuse/docs/src/commit/d173a27231a36e1a2a3af07421f5e557ae0fec46/spec.md)

Copyright DB Netz AG, licensed under Apache 2.0 (see full text in [LICENSES/Apache-2.0.txt](./LICENSES/Apache-2.0.txt))

Dot-files, cmake-files and config-files are licensed under CC0-1.0 (see full text in
[LICENSES/CC0-1.0.txt](./LICENSES/CC0-1.0.txt))