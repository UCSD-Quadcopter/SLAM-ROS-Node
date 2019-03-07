# SLAM-ROS-Node
Contains code for the SLAM Node Cluster

## packages
### `sensor_template`
A basic library to allow everyone to easily create a sensor input to a system.

- `sensor_template.h`: contains code for the class `isensor` which all sensors implement to gain sensor functionality. It contains three functions all subclasses must implement: `get_sensor_cov()`, `get_sensor_mat()`, and `get_sensor_data()`. It also contains a standard main method to be used for all sensors.
- `mat.h`: a lightweight matrix class. Essentially a lightweight wrapper class for a pointer storing the number of row and columns of the matrix as template parameters.
 
To make a sensor using this, create a class that implements isensor and implement the constructor as well as `get_sensor_cov()`, `get_sensor_mat()`, and `get_sensor_data()`. 
- `get_sensor_cov()` returns the covariance matrix which encodes the accuracy of the sensor. This is used in kalman filters and is essentially an identity matrix multiplied by some constant. We don't know the covariance of many of these sensors, so just put a standard identity matrix if you are unsure.
- `get_sensor_mat()` returns a matrix H that converts matricies in the robot state format to the sensor state format (such that \<sensor cov format\> = H\*P\*H<sup>T</sup> where P is robot covariance). In most cases, the only difference in the states is that the sensors only measure velocity, so we will just be cutting the velocity part out of the state vector. 
- `get_sensor_data()` returns the sensor data matrix, most of the time containing the velocity vector of the robot.
- for an example, see the `optflow_cam` node
- each sensor outputs to three ros topics: sensors/\<sensor name\>/cov, sensors/<\sensor name\>/meta (containing sensor matrix), and sensors/<\sensor name\>/data. `meta` and `cov` are outputted to once at the beginning of the program, while `data` is outputted to at 10 hz (should be faster now that i think about it)

### `system`
A kalman system that takes in many sensor inputs and outputs the state to one output topic. See issue #3 for more info

### `opflow_cam`
Code for a sensor that points down on the quadcopter and tracks the floor as the quadcopter moves. Also an example sensor to show how to use `sensor_template` 
