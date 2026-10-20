package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public final class ElevatorConstants {
  // Elevator CAN IDs
  public static final int PRIMARY_MOTOR_ID = 0;
  public static final int FOLLOWER_MOTOR_ID = 0;
  public static final CANBus ELEVATOR_CANBUS = new CANBus("rio");

  // Elevator Power Transmission Constants
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.44 / 2);
  public static final double GEAR_RATIO = 56 / 9;

  // Elevator Control Constants
  public static final double kG = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kP = 0.0;
  public static final double kD = 0.0;
  public static final double SUPPLY_CURRENT_LIMIT = 60.0;
  public static final double TORQUE_CURRENT_LIMIT = 80.0;

  // Elevator Motion Constants
  public static final double MAX_VELOCITY = 0.0;
  public static final double MAX_ACCELERATION = 0.0;
  public static final double MAX_JERK = 0.0;
  public static final Distance SETPOINT_TOLERANCE = Inches.of(0.1);

  // Elevator Positions
  public static final Distance STOW = Inches.of(0);
  public static final Distance CORAL_PRE_L4 = Inches.of(0);
  public static final Distance CORAL_PRE_L3 = Inches.of(0);
  public static final Distance CORAL_PRE_L2 = Inches.of(0);
  public static final Distance CORAL_L4 = Inches.of(0);
  public static final Distance CORAL_L3 = Inches.of(0);
  public static final Distance CORAL_L2 = Inches.of(0);
  public static final Distance CORAL_PRE_HANDOFF = Inches.of(0);
  public static final Distance CORAL_HANDOFF = Inches.of(0);
  public static final Distance ALGAE_GROUND = Inches.of(0);
  public static final Distance ALGAE_LOW = Inches.of(0);
  public static final Distance ALGAE_HIGH = Inches.of(0);
  public static final Distance ALGAE_TRANSIT = Inches.of(0);
  public static final Distance ALGAE_PROCESSOR = Inches.of(0);
  public static final Distance ALGAE_LOLLIPOP = Inches.of(0);
}
