// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.GyroNavX;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.Constants;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.RestrictedDriveCmd;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  // Commands
  private Command normalDriveCmd;
  private Command restrictedDriveCmd;
  private boolean isRestrictedMode = false;

  public RobotContainer() {
    gyro = new GyroNavX();
    driveSub = new DrivetrainSubsystem(new MAXSwerveModule[] {
        new MAXSwerveModule(
            Constants.DriveConstants.kFrontLeftDrivingCanId,
            Constants.DriveConstants.kFrontLeftTurningCanId,
            Constants.DriveConstants.kFrontLeftChassisAngularOffset),
        new MAXSwerveModule(
            Constants.DriveConstants.kFrontRightDrivingCanId,
            Constants.DriveConstants.kFrontRightTurningCanId,
            Constants.DriveConstants.kFrontRightChassisAngularOffset),
        new MAXSwerveModule(
            Constants.DriveConstants.kBackLeftDrivingCanId,
            Constants.DriveConstants.kBackLeftTurningCanId,
            Constants.DriveConstants.kBackLeftChassisAngularOffset),
        new MAXSwerveModule(
            Constants.DriveConstants.kBackRightDrivingCanId,
            Constants.DriveConstants.kBackRightTurningCanId,
            Constants.DriveConstants.kBackRightChassisAngularOffset)
    }, gyro);

    configureBindings();
  }

  private void configureBindings() {
    // Create normal drive command
    normalDriveCmd = new DriveCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis),
            OIConstants.kDriveDeadband));

    // Create restricted drive command (locked heading at kHeadingRestrictionDegree,
    // full X-Y control)
    restrictedDriveCmd = new RestrictedDriveCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband),
        Rotation2d.fromDegrees(Constants.DriveConstants.kHeadingRestrictionDegree)); // Use constant instead of
                                                                                     // hardcoded value

    // Set default command to normal drive
    driveSub.setDefaultCommand(normalDriveCmd);

    // Button binding: A button toggles between normal and restricted drive
    driverController.a().onTrue(
        new Command() {
          @Override
          public void initialize() {
            isRestrictedMode = !isRestrictedMode;

            if (isRestrictedMode) {
              // Switch to restricted mode
              driveSub.setDefaultCommand(restrictedDriveCmd);
              Logger.recordOutput("Drivetrain/RestrictedMode", true);
              Logger.recordOutput("Drivetrain/ModeStatus",
                  "RESTRICTED DRIVE MODE: Heading locked to " +
                      Constants.DriveConstants.kHeadingRestrictionDegree + " degrees");
            } else {
              // Switch back to normal mode
              driveSub.setDefaultCommand(normalDriveCmd);
              Logger.recordOutput("Drivetrain/RestrictedMode", false);
              Logger.recordOutput("Drivetrain/ModeStatus", "NORMAL DRIVE MODE: Full control restored");
            }
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        }.ignoringDisable(true));

    // Optional: B button to reset odometry to origin
    driverController.b().onTrue(
        new Command() {
          @Override
          public void initialize() {
            driveSub.resetPose(new edu.wpi.first.math.geometry.Pose2d());
            Logger.recordOutput("Drivetrain/OdometryReset", true);
            Logger.recordOutput("Drivetrain/ModeStatus", "Odometry reset to origin");
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        }.ignoringDisable(true));
  }

  public boolean isRestrictedMode() {
    return isRestrictedMode;
  }
}