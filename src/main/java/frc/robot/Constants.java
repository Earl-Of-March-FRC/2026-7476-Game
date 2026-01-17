// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kDriverControllerXAxis = 0;
    public static final int kDriverControllerYAxis = 1;
    public static final int kDriverControllerRotAxis = 4;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final AngularVelocity kDrivingMotorFreeSpeed = NeoMotorConstants.kFreeSpeed;
    public static final Distance kWheelDiameter = Inches.of(3);
    public static final Distance kWheelCircumference = kWheelDiameter.times(Math.PI);
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final AngularVelocity kDriveWheelFreeSpeed = RotationsPerSecond
        .of(
            kDrivingMotorFreeSpeed
                .times(kWheelCircumference)
                .div(kDrivingMotorReduction)
                .in(MultUnit.combine(RotationsPerSecond, Meters)));
  }

  public static final class NeoMotorConstants {
    public static final AngularVelocity kFreeSpeed = RotationsPerSecond.of(5676.0 / 60.0);
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.8); // Default 4.8 - Max net robot translational
                                                                            // speed
    public static final LinearVelocity kMaxWheelSpeed = MetersPerSecond.of(4.8); // Max possible speed for wheel
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(2 * Math.PI); // radians per second
    public static final LinearVelocity kBalleyPopSpeed = MetersPerSecond.of(0.8); // Max net robot translational speed
                                                                                  // when intaking algae
    // stacked on coral
    public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(3);
    public static final LinearAcceleration kMaxAccelerationPathfinding = MetersPerSecondPerSecond.of(1);
    public static final AngularVelocity kMaxAngularSpeedPathfinding = RadiansPerSecond.of(Math.PI);
    public static final AngularVelocity kMaxAngularAccelerationPathfinding = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity kBangBangTranslationalVelocity = MetersPerSecond.of(2.5);
    public static final AngularVelocity kBangBangRotationalVelocity = RadiansPerSecond
        .of((2 * Math.PI) / 10);
    public static final int kGyroDebounceThreshold = 10;

    public static final double kPIDHeadingControllerP = 3.0;
    public static final double kPIDHeadingControllerI = 0.0;
    public static final double kPIDHeadingControllerD = 0.1;
    public static final double kPIDHeadingControllerTolerance = 2.0;
    public static final Angle kHeadingRestriction = Degrees.of(45);
    // public static final PathConstraints kPathfindingConstraints = new
    // PathConstraints(kMaxSpeed.in(MetersPerSecond),
    // kMaxAccelerationPathfinding.in(MetersPerSecondPerSecond),
    // kMaxAngularSpeedPathfinding.in(RadiansPerSecond),
    // kMaxAngularAccelerationPathfinding.in(RadiansPerSecond));

    // Chassis configuration
    public static final Distance kTrackWidth = Inches.of(26.5);
    // Distance between centers of right and left wheels on robot
    public static final Distance kWheelBase = Inches.of(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
        new Translation2d(kWheelBase.div(2).in(Meters), -kTrackWidth.div(2).in(Meters)),
        new Translation2d(-kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
        new Translation2d(-kWheelBase.div(2).in(Meters), -kTrackWidth.div(2).in(Meters)));

    // Angular offsets of the modules relative to the chassis in radians
    public static final Angle kFrontLeftChassisAngularOffset = Radians.of(-Math.PI / 2);
    public static final Angle kFrontRightChassisAngularOffset = Radians.of(0);
    public static final Angle kBackLeftChassisAngularOffset = Radians.of(Math.PI);
    public static final Angle kBackRightChassisAngularOffset = Radians.of(Math.PI / 2);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kBackLeftDrivingCanId = 7;
    public static final int kBackRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kBackLeftTurningCanId = 3;
    public static final int kBackRightTurningCanId = 4;

    public static final boolean kGyroReversed = false;
  }

}