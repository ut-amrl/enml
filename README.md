# Episodic non-Markov Localization

[![Build Status](https://travis-ci.com/ut-amrl/enml.svg?token=rBLDT1qXfkKmkTerGLzY&branch=master)](https://travis-ci.com/ut-amrl/enml)

Implementation in C++ of Episodic non-Markov Localization [[pdf]](https://www.joydeepb.com/Publications/ras_episodic_nonmarkov_localization.pdf).
Please cite this work using the following bibtex:
```
@article{biswas2016episodic,
  author = { Joydeep Biswas and Manuela M. Veloso },
  url = { https://www.joydeepb.com/Publications/ras_episodic_nonmarkov_localization.pdf },
  doi = { 10.1016/j.robot.2016.09.005 },
  issn = { 0921-8890 },
  year = { 2017 },
  pages = { 162 - 176 },
  volume = { 87 },
  journal = { Robotics and Autonomous Systems  },
  title = { Episodic non-Markov localization },
}
```

## Compiling
1. Run `./InstallPackages` to install the dependencies on *ubuntu >=14.04 .
1. Add the working directory to the `ROS_PACKAGE_PATH` environment variable with:

   ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```
1. Run `make`

## Configuration
This project uses the `config-reader`.

The main configuration file can be found at `config/enml.lua`, which specifies parameters for the ENML algorithm. including the map name used to localize.

In addition, a file called `config/robot.lua` must be specified, which configures the input topics, robot name, etc. An example can be found at `config/robot.lua.example`.

## Usage

To run enml, simply run `./bin/enml`

While running, you may want to visualize what's going on. To do this, add the `-d1` flag to the run command, which will cause enml to publish visualization messages while running.

To see the resultant visualization, there are 2 options:
#### Localization GUI
Obtain and setup the `ut-amrl/cobot` repo.
Then run `./bin/localization_gui`, with the `map-name` parameter matching the map used for enml localization. This will automatically

#### WebRViz
Obtain and setup the `ut-amrl/robofleet-deployment-server` repo.

Run the websocket `./bin/websocket`. 

Open the webrviz html file in browser, and connect to localhost.