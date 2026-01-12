package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.IndexerConstants;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
          / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  // public static final class ArmConfigs {
  // public static final SparkMaxConfig armConfig = new SparkMaxConfig();

  // static {

  // armConfig
  // .idleMode(IdleMode.kBrake) // Set to kBrake to hold position when not moving
  // .smartCurrentLimit(40); // Adjust current limit as needed

  // armConfig.encoder
  // .positionConversionFactor(ArmConstants.kPositionConversionFactor) // Radians
  // .velocityConversionFactor(ArmConstants.kVelocityConversionFactor); // Radians
  // per second

  // armConfig.closedLoop
  // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  // // Upward position PID controller is slot 0
  // .pidf(ArmConstants.kPPositionController, ArmConstants.kIPositionController,
  // ArmConstants.kDPositionController, ArmConstants.kPositionFF,
  // ClosedLoopSlot.kSlot0)

  // // Downward position PID controller is slot 1
  // // .pidf(ArmConstants.kPDownPositionController,
  // // ArmConstants.kIDownPositionController,
  // // ArmConstants.kDDownPositionController, ArmConstants.kDownPositionFF,
  // // ClosedLoopSlot.kSlot1)
  // .outputRange(-1, 1);
  // }
  // }

  // public static final class IntakeConfigs {
  // public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

  // static {
  // intakeConfig
  // .idleMode(IdleMode.kCoast) // Set to kBrake to hold position when not moving
  // .smartCurrentLimit(30); // Adjust current limit as needed

  // intakeConfig.encoder
  // .positionConversionFactor(IntakeConstants.kPositionConversionFactor *
  // IntakeConstants.kMotorReduction)
  // .velocityConversionFactor(IntakeConstants.kVelocityConversionFactor *
  // IntakeConstants.kMotorReduction);
  // }
  // }

  // public static final class IndexerConfigs {

  // public static final SparkMaxConfig indexerConfig = new SparkMaxConfig();

  // static {
  // indexerConfig.idleMode(IdleMode.kBrake);
  // indexerConfig.smartCurrentLimit(40);
  // indexerConfig.encoder
  // .velocityConversionFactor(IndexerConstants.kWheelDiameterMeters * Math.PI
  // / IndexerConstants.kMotorReduction / 60);
  // }
  // }

  // public static final class LauncherConfigs {
  // public static final SparkMaxConfig frontLauncherConfig = new
  // SparkMaxConfig();
  // public static final SparkMaxConfig backLauncherConfig = new SparkMaxConfig();

  // static {
  // frontLauncherConfig
  // .idleMode(IdleMode.kCoast)
  // .smartCurrentLimit(50)
  // .voltageCompensation(10);
  // frontLauncherConfig.encoder
  // .positionConversionFactor(1)
  // .velocityConversionFactor(LauncherConstants.kVelocityConversionFactor);
  // frontLauncherConfig.closedLoop
  // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  // .pidf(LauncherConstants.kPVelocityControllerHigh,
  // LauncherConstants.kIVelocityControllerHigh,
  // LauncherConstants.kDVelocityControllerHigh,
  // LauncherConstants.frontKVelocityFFHigh, LauncherConstants.kSlotHigh)
  // .pidf(LauncherConstants.kPVelocityControllerLow,
  // LauncherConstants.kIVelocityControllerLow,
  // LauncherConstants.kDVelocityControllerLow,
  // LauncherConstants.frontKVelocityFFLow, LauncherConstants.kSlotLow)
  // .outputRange(-1, 1);
  // }

  // static {
  // backLauncherConfig
  // .idleMode(IdleMode.kCoast)
  // .smartCurrentLimit(50)
  // .voltageCompensation(10);
  // backLauncherConfig.encoder
  // .positionConversionFactor(1)
  // .velocityConversionFactor(LauncherConstants.kVelocityConversionFactor);
  // backLauncherConfig.closedLoop
  // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  // .pidf(LauncherConstants.kPVelocityControllerHigh,
  // LauncherConstants.kIVelocityControllerHigh,
  // LauncherConstants.kDVelocityControllerHigh,
  // LauncherConstants.backKVelocityFFHigh, LauncherConstants.kSlotHigh)
  // .pidf(LauncherConstants.kPVelocityControllerLow,
  // LauncherConstants.kIVelocityControllerLow,
  // LauncherConstants.kDVelocityControllerLow,
  // LauncherConstants.backKVelocityFFLow, LauncherConstants.kSlotLow)
  // .outputRange(-1, 1);
  // }
  // }
}