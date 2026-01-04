package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
  /**
   * Get the measured angle from the gyro.
   * 
   * @return The measured angle
   */
  public Rotation2d getRotation2d();

  /**
   * Gets the rotational velocity in degrees per second.
   */
  public double getRate();

  /**
   * Calibrates the gyro.
   */
  public default void calibrate() {
  }

  /**
   * Resets the gyro to a set angle.
   * 
   * @param angle The angle to set the gyro to
   */
  public default void setAngle(Rotation2d angle) {
  }

  /**
   * Get if the gyro is connected.
   * 
   * @return {@code true} if the gyro is connected
   */
  public boolean isConnected();
}