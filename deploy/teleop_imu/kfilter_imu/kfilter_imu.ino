/*
 * teleop_imu_starter
 *
 * Starter sketch for the IMU teleoperation firmware.
 * Write your Arduino code here (this folder is your firmware workspace:
 * extra .ino/.h/.cpp files placed in this folder are compiled together).
 *
 * The MuJoCo game (deploy/deploy_mujoco/deploy_mujoco_imu_obstacles_starter.py)
 * talks to this board over the serial port.
 */

void setup()
{
  Serial.begin(115200);

  // Initialise your sensors here
}

void loop()
{
  // Read the IMU and answer the commands from the PC here
}
