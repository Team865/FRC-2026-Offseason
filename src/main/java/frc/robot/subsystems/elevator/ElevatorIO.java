package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean primaryConnected = false;
    public boolean followerConnected = false;
    public Distance position = Meters.zero();
    public Distance targetPosition = Meters.zero();
    public LinearVelocity velocity = MetersPerSecond.zero();
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }

  public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}

  public default void setVolts(double volts) {}

  public default void setPosition(Distance position) {}

  public default boolean seedPosition(Distance position) {
    return true;
  }

  public default void setControlConstants(
      double kG, double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(
      double maxVelocity, double maxAcceleration, double maxJerk) {}

  public default void stop() {}
}
