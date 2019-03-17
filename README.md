# SLAMTEC RPLiDAR A2 C++ Library
## API
### 1. *RPLidar* class
##### bool RPLidar::init_port(const char* serial_path)
* Initializes internal buffers and Serial Port.
* Stops scans and motor.
* returns the success of the operation

##### bool RPLidar::start()
* Starts the motor of the lidar.
* Waits 2 seconds for its speed to stabilize.
* Starts Express Scans (see the [RPLidar A1&A2 Communication protocol](https://www.robotshop.com/media/files/pdf2/rpk-02-communication-protocol.pdf) note).
* returns true in case of success for both the motor start and scan start operations

