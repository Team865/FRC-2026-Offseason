package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIO.ElevatorIOInputsAutoLogged inputs =
      new ElevatorIO.ElevatorIOInputsAutoLogged();

  // do this later
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.56);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.24);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 4.44);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.07);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 100);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/MaxVel", ElevatorConstants.MAX_VELOCITY);
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/maxAccel", ElevatorConstants.MAX_ACCELERATION);
  private final LoggedTunableNumber maxJerk =
      new LoggedTunableNumber("Elevator/maxJerk", ElevatorConstants.MAX_JERK);

  private Distance targetPosition = ElevatorConstants.STOW;

  public Elevator(ElevatorIO io) {
    this.io = io;

    io.setControlConstants(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    io.setMotionProfile(maxVelocity.get(), maxAcceleration.get(), maxJerk.get());
  }

  public Command setTargetPosition(Distance position) {
    return this.runOnce(() -> this.targetPosition = position);
  }

  public Trigger atSetpoint(Distance targetPosition) {
    return new Trigger(
        () -> inputs.position.isNear(targetPosition, ElevatorConstants.SETPOINT_TOLERANCE));
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () -> inputs.position.isNear(inputs.targetPosition, ElevatorConstants.SETPOINT_TOLERANCE));
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Distance getPosition() {
    return inputs.position;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    io.setPosition(targetPosition);
  }
}
