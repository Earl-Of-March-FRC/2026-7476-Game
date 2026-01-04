// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.Constants;
import frc.robot.commands.DriveCmd;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
  // Declare subsystems and controllers at the class level
  public final DrivetrainSubsystem driveSub;
  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    gyro = new Gyro();
    driveSub = new DrivetrainSubsystem(new MAXSwerveModule[] {
        new MAXSwerveModule(Constants.DriveConstants.kFrontLeftDrivingCanId,
            Constants.DriveConstants.kFrontLeftTurningCanId, Constants.DriveConstants.kFrontLeftChassisAngularOffset), // FL
        new MAXSwerveModule(Constants.DriveConstants.kFrontRightDrivingCanId,
            Constants.DriveConstants.kFrontRightTurningCanId, Constants.DriveConstants.kFrontRightChassisAngularOffset), // FR
        new MAXSwerveModule(Constants.DriveConstants.kBackLeftDrivingCanId,
            Constants.DriveConstants.kBackLeftTurningCanId, Constants.DriveConstants.kBackLeftChassisAngularOffset), // BL
        new MAXSwerveModule(Constants.DriveConstants.kBackRightDrivingCanId,
            Constants.DriveConstants.kBackRightTurningCanId, Constants.DriveConstants.kBackRightChassisAngularOffset) // BR
    }, gyro);

    configureBindings();
  }

  private void configureBindings() {
    driveSub.setDefaultCommand(
        new DriveCmd(
            driveSub,
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
                OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
                OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis),
                OIConstants.kDriveDeadband)));
  }
}
