// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * Command for restricted swerve driving at a locked angle.
 * This command locks the robot to move only along a specific angle (e.g., 45
 * degrees)
 * with one degree of freedom - forward/backward along that line.
 */
public class RestrictedDriveCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> forwardSupplier;
  private final Rotation2d lockedAngle;
  private final double maxSpeed;

  /**
   * Creates a new RestrictedDriveCmd.
   * 
   * @param driveSub        The drivetrain subsystem
   * @param forwardSupplier Supplier for forward/backward input (-1 to 1)
   * @param lockedAngle     The angle to lock movement to (field-relative)
   * @param maxSpeed        Maximum speed in meters per second
   */
  public RestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> forwardSupplier,
      Rotation2d lockedAngle,
      double maxSpeed) {
    this.driveSub = driveSub;
    this.forwardSupplier = forwardSupplier;
    this.lockedAngle = lockedAngle;
    this.maxSpeed = maxSpeed;

    addRequirements(driveSub);
  }

  /**
   * Convenience constructor with default speed limit.
   */
  public RestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> forwardSupplier,
      Rotation2d lockedAngle) {
    this(driveSub, forwardSupplier, lockedAngle, DriveConstants.kMaxSpeedMetersPerSecond);
  }

  @Override
  public void initialize() {
    // Auto-rotate the robot to the locked angle when command starts
    driveSub.setTargetHeading(lockedAngle);
  }

  @Override
  public void execute() {
    // Get forward/backward input
    double forwardInput = forwardSupplier.get();
    double speed = forwardInput * maxSpeed;

    // Calculate velocity components along the locked angle
    double vx = speed * lockedAngle.getCos();
    double vy = speed * lockedAngle.getSin();

    // Get rotational velocity to maintain heading
    double omega = driveSub.getHeadingCorrectionOmega(lockedAngle);

    // Apply the chassis speeds (field-relative)
    driveSub.runVelocity(new ChassisSpeeds(vx, vy, omega), true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
    driveSub.clearTargetHeading();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}