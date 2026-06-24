// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
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
    Logger.recordOutput("Commands/Carrot/Initialized", true);
  }

  @Override
  public void execute() {
    Optional<Pose3d> tagPoseRobotRelative = driveSub.getTagPoseRobotRelative(DriveConstants.kCarrotOnAStickTagId);
    Logger.recordOutput("Commands/Carrot/SeesTag", tagPoseRobotRelative.isPresent());

    if (tagPoseRobotRelative.isEmpty()) {
      // driveSub
      // .runVelocity(
      // new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.zero(),
      // DriveConstants.kNoTagTurnSpeed.times(lastAngularVelocitySign)),
      // false, false, false);
      return;
    }

    Pose2d targetPoseError = tagPoseRobotRelative.get().transformBy(DriveConstants.kCarrotOnAStickTagToTargetPose)
        .toPose2d();

    LinearVelocity xSpeed = SwerveConfig.kMaxSpeed.times(xController.calculate(targetPoseError.getX()));
    LinearVelocity ySpeed = SwerveConfig.kMaxSpeed.times(yController.calculate(targetPoseError.getY()));
    AngularVelocity omega = SwerveConfig.kMaxAngularSpeed
        .times(headingController.calculate(targetPoseError.getRotation().getRadians()));

    lastAngularVelocitySign = (int) Math.signum(omega.in(RadiansPerSecond));

    // driveSub
    // .runVelocity(
    // new ChassisSpeeds(xSpeed,
    // ySpeed,
    // omega),
    // false, false, false);

    Logger.recordOutput("Commands/Carrot/xSpeed", xSpeed);
    Logger.recordOutput("Commands/Carrot/ySpeed", ySpeed);
    Logger.recordOutput("Commands/Carrot/omega", omega);
    Logger.recordOutput("Commands/Carrot/targetPoseError", targetPoseError);
    Logger.recordOutput("Commands/Carrot/tagPoseRobotRelative", tagPoseRobotRelative.get());
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/Carrot/Initialized", false);
    System.out.println("vndkjsnvkj");
    driveSub.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
