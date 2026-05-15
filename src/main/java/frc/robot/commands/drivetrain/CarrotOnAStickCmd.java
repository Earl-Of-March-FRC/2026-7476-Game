// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Optional;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.SwerveConfig;

public class CarrotOnAStickCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private int lastAngularVelocitySign = 0;

  private final PIDController xController = new PIDController(DriveConstants.kPIDXControllerP,
      DriveConstants.kPIDXControllerI, DriveConstants.kPIDXControllerD);
  private final PIDController yController = new PIDController(DriveConstants.kPIDYControllerP,
      DriveConstants.kPIDYControllerI, DriveConstants.kPIDYControllerD);
  private final PIDController headingController = new PIDController(DriveConstants.kPIDHeadingControllerP,
      DriveConstants.kPIDHeadingControllerI, DriveConstants.kPIDHeadingControllerD);

  /** Creates a new CarrotOnAStickCmd. */
  public CarrotOnAStickCmd(DrivetrainSubsystem driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    headingController.setSetpoint(0);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Optional<Pose3d> tagPoseRobotRelative = driveSub.getTagPoseRobotRelative(DriveConstants.kCarrotOnAStickTagId);

    if (tagPoseRobotRelative.isEmpty()) {
      driveSub
          .runVelocity(
              new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.zero(),
                  DriveConstants.kNoTagTurnSpeed.times(lastAngularVelocitySign)),
              false, false, false);
    }

    Pose2d targetPoseError = tagPoseRobotRelative.get().transformBy(DriveConstants.kCarrotOnAStickTagToTargetPose)
        .toPose2d();

    double xSpeed = xController.calculate(targetPoseError.getX());
    double ySpeed = yController.calculate(targetPoseError.getY());
    double omega = headingController.calculate(targetPoseError.getRotation().getRadians());

    lastAngularVelocitySign = (int) Math.signum(omega);

    driveSub
        .runVelocity(
            new ChassisSpeeds(SwerveConfig.kMaxSpeed.times(xSpeed),
                SwerveConfig.kMaxSpeed.times(ySpeed),
                SwerveConfig.kMaxAngularSpeed.times(omega)),
            false, false, false);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
