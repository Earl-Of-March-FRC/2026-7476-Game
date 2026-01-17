// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * Command for restricted swerve driving with locked heading.
 * This command locks the robot's heading to a specific angle (e.g., 45 degrees)
 * while still allowing full X-Y translational movement with the joystick.
 * The robot will automatically rotate to maintain the locked heading.
 */
public class RestrictedDriveCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Rotation2d lockedAngle;
  private final LinearVelocity maxSpeed;

  /**
   * Creates a new RestrictedDriveCmd.
   * 
   * @param driveSub    The drivetrain subsystem
   * @param xSupplier   Supplier for X-axis input (-1 to 1, field-relative)
   * @param ySupplier   Supplier for Y-axis input (-1 to 1, field-relative)
   * @param lockedAngle The angle to lock the robot's heading to (field-relative)
   * @param maxSpeed    Maximum speed in meters per second
   */
  public RestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Rotation2d lockedAngle,
      LinearVelocity maxSpeed) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.lockedAngle = lockedAngle;
    this.maxSpeed = maxSpeed;
    addRequirements(driveSub);
  }

  /**
   * Convenience constructor with default speed limit.
   */
  public RestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Rotation2d lockedAngle) {
    this(driveSub, xSupplier, ySupplier, lockedAngle, DriveConstants.kMaxSpeed);
  }

  @Override
  public void initialize() {
    // Set the target heading for the robot to maintain
    driveSub.setTargetHeading(lockedAngle);
  }

  @Override
  public void execute() {
    // Get X and Y joystick inputs (full 2D control)
    LinearVelocity xVel = maxSpeed.times(xSupplier.get());
    LinearVelocity yVel = maxSpeed.times(ySupplier.get());

    // Get rotational velocity to maintain the locked heading
    AngularVelocity omega = driveSub.getHeadingCorrectionOmega(lockedAngle);

    // Apply chassis speeds (field-relative with locked heading)
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega), true);
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