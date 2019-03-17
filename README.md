# SLAMTEC RPLiDAR A2 C++ Library
## API
### - *RPLidar* class
##### bool RPLidar::init_port(const char* serial_path)
* Initializes internal buffers and Serial Port.
* Stops scans and motor.
* Returns the success of the operation.

##### bool RPLidar::start()
* Starts the motor of the lidar.
* Waits 2 seconds for its speed to stabilize.
* Starts Express Scans (see the [RPLidar A1&A2 Communication protocol](https://www.robotshop.com/media/files/pdf2/rpk-02-communication-protocol.pdf) note).
* Returns true in case of success for both the motor start and scan start operations.

##### bool RPLidar::stop()
* Stops current Scan.
* Stops the motor of the lidar(tries rp_values::NUMBER_PWM_TRIES times as we do not have any information on the success).
* Returns true in case of success for both the motor stop and scan stop operations.

##### bool RPLidar::close()
* Closes file descriptor for the used serial port.
* Returns the success of the file close operation.

