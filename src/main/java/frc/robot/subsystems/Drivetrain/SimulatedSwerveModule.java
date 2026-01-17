package frc.robot.subsystems.Drivetrain;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SimulatedSwerveModule implements SwerveModule {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController turnController;

  public SimulatedSwerveModule(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    drivePID = new PIDController(ModuleConstants.kDrivingPSim, ModuleConstants.kDrivingISim,
        ModuleConstants.kDrivingDSim);
    driveFeedforward = new SimpleMotorFeedforward(0.0, ModuleConstants.kDrivingFFSim);
    turnController = new PIDController(ModuleConstants.kTurningPSim, ModuleConstants.kTurningISim,
        ModuleConstants.kTurningDSim);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    driveMotor = moduleSimulation.useGenericMotorControllerForDrive();
    turnMotor = moduleSimulation.useGenericControllerForSteer();
  }

  @Override
  public SwerveModuleState getState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getState'");
  }

  @Override
  public SwerveModulePosition getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDesiredState'");
  }

  @Override
  public void resetEncoders() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetEncoders'");
  }

}
