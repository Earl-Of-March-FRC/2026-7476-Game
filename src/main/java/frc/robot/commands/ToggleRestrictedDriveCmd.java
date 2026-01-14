// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * Command that toggles between normal drive mode and restricted drive mode.
 * When activated, locks the robot to 45-degree movement.
 * When deactivated, returns to normal driving.
 */
public class ToggleRestrictedDriveCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Supplier<Double> omegaSupplier;
  private final Supplier<Double> forwardSupplier;
  private final Rotation2d restrictedAngle;

  private Command normalDriveCmd;
  private Command restrictedDriveCmd;
  private boolean isRestricted = false;

  /**
   * Creates a new ToggleRestrictedDriveCmd.
   * 
   * @param driveSub        The drivetrain subsystem
   * @param xSupplier       X-axis input supplier for normal drive
   * @param ySupplier       Y-axis input supplier for normal drive
   * @param omegaSupplier   Rotation input supplier for normal drive
   * @param forwardSupplier Forward/backward input for restricted drive
   * @param restrictedAngle The angle to restrict movement to (default: 45
   *                        degrees)
   */
  public ToggleRestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier,
      Supplier<Double> forwardSupplier,
      Rotation2d restrictedAngle) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.forwardSupplier = forwardSupplier;
    this.restrictedAngle = restrictedAngle;

    // Create the two drive commands
    normalDriveCmd = new DriveCmd(driveSub, xSupplier, ySupplier, omegaSupplier);
    restrictedDriveCmd = new RestrictedDriveCmd(driveSub, forwardSupplier, restrictedAngle);

    addRequirements(driveSub);
  }

  /**
   * Convenience constructor with default 45-degree angle.
   */
  public ToggleRestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier,
      Supplier<Double> forwardSupplier) {
    this(driveSub, xSupplier, ySupplier, omegaSupplier, forwardSupplier,
        Rotation2d.fromDegrees(45));
  }

  @Override
  public void initialize() {
    // Toggle the mode
    isRestricted = !isRestricted;

    if (isRestricted) {
      // Switch to restricted mode
      restrictedDriveCmd.schedule();
      Logger.recordOutput("Drivetrain/RestrictedMode", true);
      Logger.recordOutput("Drivetrain/ModeStatus",
          "Restricted Drive Mode ENABLED - Locked to " + restrictedAngle.getDegrees() + " degrees");
    } else {
      // Switch back to normal mode
      normalDriveCmd.schedule();
      Logger.recordOutput("Drivetrain/RestrictedMode", false);
      Logger.recordOutput("Drivetrain/ModeStatus", "Restricted Drive Mode DISABLED - Normal driving");
    }
  }

  @Override
  public boolean isFinished() {
    return true; // Command finishes immediately after toggling
  }

  public boolean isRestricted() {
    return isRestricted;
  }
}