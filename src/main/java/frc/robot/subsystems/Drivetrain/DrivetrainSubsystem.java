// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4]; // FL, FR, BL, BR
  private static final SwerveDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;
  public final Gyro gyro;
  public boolean gyroDisconnected = false;
  public boolean isFieldRelative = true; // Default to field-relative driving
  private int gyroDisconnectCounter = 0; // Debouncer counter

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem(MAXSwerveModule[] modules, Gyro gyro) {
    for (int i = 0; i < modules.length; i++) {
      this.modules[i] = modules[i];
    }
    this.gyro = gyro;
  }

  public void runVelocity(ChassisSpeeds speeds, Boolean isFieldRelative) {
    // If the speeds are field-relative, convert them to robot-relative speeds
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          gyro.getRotation2d());
    }
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Desaturate the wheel speeds to ensure they are within the maximum speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxWheelSpeedMetersPerSecond);

    // Set the desired state for each swerve module
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    // Log the desired states of the swerve modules to the logger
    Logger.recordOutput("Swerve/Module/Setpoint", states);
    Logger.recordOutput("Swerve/Gyro", gyro.getRotation2d());
  }

  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, isFieldRelative);
  }

  @Override
  public void periodic() {
    // Check if the gyro is connected
    if (!gyro.isConnected()) { // Assuming gyro has an isConnected() method
      gyroDisconnectCounter++;
      if (gyroDisconnectCounter >= DriveConstants.kGyroDebounceThreshold) {
        gyroDisconnected = true;
        isFieldRelative = false; // Disable field-relative driving
      }
    } else {
      gyroDisconnectCounter = 0; // Reset counter if gyro is connected
      gyroDisconnected = false;
    }

    // Log the gyro status
    Logger.recordOutput("Drivetrain/GyroDisconnected", gyroDisconnected);
    Logger.recordOutput("Drivetrain/IsFieldRelative", isFieldRelative);

    // Existing periodic functionality
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
      positions[i] = modules[i].getPosition();
    }

    Logger.recordOutput("Swerve/Module/State", states);
    Logger.recordOutput("Swerve/Module/Position", positions);
  }
}
