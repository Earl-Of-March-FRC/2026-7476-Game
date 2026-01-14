// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4]; // FL, FR, BL, BR
  private static final SwerveDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;
  public final Gyro gyro;
  public boolean gyroDisconnected = false;
  public boolean isFieldRelative = true;
  private int gyroDisconnectCounter = 0;

  // Odometry
  private final SwerveDrivePoseEstimator poseEstimator;

  // Heading control for restricted driving
  private Rotation2d targetHeading = null;
  private final PIDController headingController;

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem(MAXSwerveModule[] modules, Gyro gyro) {
    for (int i = 0; i < modules.length; i++) {
      this.modules[i] = modules[i];
    }
    this.gyro = gyro;

    // Initialize pose estimator with starting pose
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d());

    // Initialize heading controller for auto-rotation
    headingController = new PIDController(Constants.DriveConstants.kPIDHeadingControllerP,
        Constants.DriveConstants.kPIDHeadingControllerI, Constants.DriveConstants.kPIDHeadingControllerD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Math.toRadians(Constants.DriveConstants.kPIDHeadingControllerTollerance));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxWheelSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  public void runVelocity(ChassisSpeeds speeds, Boolean isFieldRelative) {
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          gyro.getRotation2d());
    }
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxWheelSpeedMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    Logger.recordOutput("Swerve/Module/Setpoint", states);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, isFieldRelative);
  }

  /**
   * Gets the current pose of the robot from odometry.
   * 
   * @return The current estimated pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to a specific pose.
   * 
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Gets all module positions.
   * 
   * @return Array of module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets all module states.
   * 
   * @return Array of module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Sets a target heading for the robot to maintain.
   * Used in restricted drive mode.
   * 
   * @param heading The target heading
   */
  public void setTargetHeading(Rotation2d heading) {
    targetHeading = heading;
  }

  /**
   * Clears the target heading.
   */
  public void clearTargetHeading() {
    targetHeading = null;
  }

  /**
   * Gets the omega correction to maintain the target heading.
   * 
   * @param desiredHeading The desired heading to maintain
   * @return The omega correction in radians per second
   */
  public double getHeadingCorrectionOmega(Rotation2d desiredHeading) {
    Rotation2d currentHeading = gyro.getRotation2d();
    double error = desiredHeading.minus(currentHeading).getRadians();
    return headingController.calculate(0, error);
  }

  /**
   * Gets the current chassis speeds.
   * 
   * @return Current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {
    // Check gyro connection with debouncing
    if (!gyro.isConnected()) {
      gyroDisconnectCounter++;
      if (gyroDisconnectCounter >= DriveConstants.kGyroDebounceThreshold) {
        gyroDisconnected = true;
        isFieldRelative = false;
      }
    } else {
      gyroDisconnectCounter = 0;
      gyroDisconnected = false;
    }

    // Update odometry
    poseEstimator.update(gyro.getRotation2d(), getModulePositions());

    // Get current states and positions
    SwerveModuleState[] states = getModuleStates();
    SwerveModulePosition[] positions = getModulePositions();
    Pose2d currentPose = getPose();

    // Log everything
    Logger.recordOutput("Drivetrain/GyroDisconnected", gyroDisconnected);
    Logger.recordOutput("Drivetrain/IsFieldRelative", isFieldRelative);
    Logger.recordOutput("Drivetrain/Pose", currentPose);
    Logger.recordOutput("Drivetrain/Rotation", gyro.getRotation2d().getDegrees());
    Logger.recordOutput("Swerve/Module/State", states);
    Logger.recordOutput("Swerve/Module/Position", positions);

    if (targetHeading != null) {
      Logger.recordOutput("Drivetrain/TargetHeading", targetHeading.getDegrees());
    }
  }
}