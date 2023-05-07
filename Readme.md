# Sensor Fusion for Automotive Dead Reckoning

### Summary
This project aims to build a navigation stack using 2 different sensors - GPS and IMU, understand their relative strengths & drawbacks, and explore sensor fusion.

One Vectornav VN-100 IMU module was used to collect the orientation, angular velocity, linear acceleration, and magnetometer data, along with a USB based GNSS puck to collect the navigation data. Two sets of data were collected: one going in circles at the Forsyth Circle for magnetometer calibration, and one for a longer duration on a planned path of about 2-3km, with the same start and end points. The IMU and the GNSS puck, in both cases were taped inside and on the roof of the NUance car respectively so that they do not move for the whole duration. The data was collected into .bag files and corrected & analyzed using MATLAB.

The following has been performed in MATLAB:
1. Magnetometer Calibration
2. Sensor Fusion to estimate the vehicle's heading (yaw)
3. Forward velocity estimation
4. Dead Reckoning with IMU

## Run the code
Launch both GPS and IMU drivers simultaneously through master.launch file using: 
roslaunch sensor_fusion master.launch gps_port:=<your GPS port> imu_port:=<your IMU port> 

Each driver can be individualy launched if required using:
GPS driver: roslaunch gps_driver driver.launch port:=<your GPS port>
IMU driver: roslaunch imu_driver driver.launch port:=<your IMU port>

## Analysis & Report can be found in /src/analysis folder
