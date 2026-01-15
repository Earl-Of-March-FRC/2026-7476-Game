// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * Command that toggles between normal drive mode and restricted drive mode.
 * When activated, locks the robot's heading to a specific angle while allowing
 * full X-Y movement.
 * When deactivated, returns to normal driving with rotational control.
 */
public class ToggleRestrictedDriveCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Supplier<Double> omegaSupplier;
  private final Rotation2d restrictedAngle;

  private Command normalDriveCmd;
  private Command restrictedDriveCmd;
  private boolean isRestricted = false;

  /**
   * Creates a new ToggleRestrictedDriveCmd.
   * 
   * @param driveSub        The drivetrain subsystem
   * @param xSupplier       X-axis input supplier
   * @param ySupplier       Y-axis input supplier
   * @param omegaSupplier   Rotation input supplier for normal drive
   * @param restrictedAngle The angle to lock the robot's heading to
   */
  public ToggleRestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier,
      Rotation2d restrictedAngle) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.restrictedAngle = restrictedAngle;

    // Create the two drive commands
    normalDriveCmd = new DriveCmd(driveSub, xSupplier, ySupplier, omegaSupplier);
    restrictedDriveCmd = new RestrictedDriveCmd(driveSub, xSupplier, ySupplier, restrictedAngle);

    addRequirements(driveSub);
  }

  /**
   * Convenience constructor using the constant from DriveConstants.
   */
  public ToggleRestrictedDriveCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier) {
    this(driveSub, xSupplier, ySupplier, omegaSupplier,
        Rotation2d.fromDegrees(DriveConstants.kHeadingRestrictionDegree));
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
          "Restricted Drive Mode ENABLED - Heading locked to " + restrictedAngle.getDegrees() + " degrees");
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