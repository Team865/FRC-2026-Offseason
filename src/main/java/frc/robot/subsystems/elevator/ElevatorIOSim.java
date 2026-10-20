package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {

  private double appliedVolts = 0.0;
  private Distance targetPosition = Meters.zero();
  private boolean voltageOverridesTarget = false;

  private final DCMotorSim motorSim;

  private final ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));

  public ElevatorIOSim(DCMotor motor, ElevatorConstants constants) {
    this.motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV, constants.kA), motor);

    setControlConstants(
        constants.kS, constants.kV, constants.kA, constants.kP, constants.kI, constants.kD);
    setMotionProfile(constants.MAX_VELOCITY, constants.MAX_ACCELERATION, constants.MAX_JERK);
  }

  @Override
  public void setVolts(Voltage volts) {
    setClampedVolts(volts);
    voltageOverridesTarget = true;
  }

  @Override
  public void setPosition(Distance position) {
    targetPosition = position;
    voltageOverridesTarget = false;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    voltageOverridesTarget = true;
  }

  private void setClampedVolts(Voltage volts) {
    appliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
  }

  @Override
  public void setControlConstants(
      double kG, double kS, double kV, double kA, double kP, double kD) {
    feedforwardController.setKg(kG);
    feedforwardController.setKs(kS);
    feedforwardController.setKa(kA);
    feedforwardController.setKv(kV);

    pidController.setP(kP);
    pidController.setD(kD);
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration, double maxJerk) {
    pidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }
}
