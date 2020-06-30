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

### Visualization

While running, you may want to visualize what's going on. To do this, add the `-d1` flag to the run command, which will cause enml to publish visualization messages while running.

To see the resultant visualization, there are 2 options:
#### VectorDisplay
Obtain and setup the `ut-amrl/vector_display` repo.
Then run `./bin/vector_display`, optionally with the `map-name` parameter matching the map used for enml localization. This will automatically listen to the same topics `enml` is publishing, and you will see the visualization in the localization gui's window.

#### WebRViz
Obtain and setup the `ut-amrl/robofleet-deployment-server` repo.

Run the websocket `./bin/websocket`. 

Open the webrviz html file in browser, and connect to localhost. This will automatically listen to the same topics `enml` is publishing, and you will see the visualization in the web rviz's viewport.

## Example

![EnML Example](example.png)

1. Download the example ROS bag file of the UT-Jackal navigating from GDC to AHG: [2020-06-03-18-51-39.bag](https://drive.google.com/file/d/1GrQ3982jt0dSS8Yw0h-hxMvZXgpmObWY/view?usp=sharing)
1. Edit `config/robot.lua` to match the UT Jackal's configuration:
    ```
    RobotConfig = {
      name = "ut-jackal";
      scan_topic = "/scan";
      pointcloud_topic = "";
      odometry_topic = "/jackal_velocity_controller/odom";
      initialpose_topic = "/initialpose";
    };
    ```
1. Edit the `config/enml.lua` to set the initial pose in the `ut-jackal` section:
    ```
    ...
    if RobotConfig.name=="ut-jackal" then
      ...
      -- 2020-06-03-18-51-39.bag
      enml.map_name = "UT_Campus";
      enml.starting_loc_x = 131;
      enml.starting_loc_y = -245;
      enml.starting_angle = deg2rad(-85);
    ...
    ```
1. Run vector_display
1. Run Enml in offline bag replay mode with the example bag file:
    ```
    ./bin/enml -d1 -b 2020-06-03-18-51-39.bag
    ```
