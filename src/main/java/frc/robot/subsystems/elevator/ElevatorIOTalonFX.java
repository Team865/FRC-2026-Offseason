package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Talon Objects
  private final TalonFX primaryTalon;
  private final TalonFX followerTalon;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Control Signals
  private final VoltageOut voltageRequest =
      new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final Debouncer masterConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  // Status Signals
  private final StatusSignal<Angle> position;

  private final StatusSignal<AngularVelocity> primaryVelocity;
  private final StatusSignal<Voltage> primaryAppliedVolts;
  private final StatusSignal<Current> primaryTorqueCurrent;
  private final StatusSignal<Current> primarySupplyCurrent;
  private final StatusSignal<Temperature> primaryTemp;

  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  public ElevatorIOTalonFX() {
    primaryTalon =
        new TalonFX(ElevatorConstants.PRIMARY_MOTOR_ID, ElevatorConstants.ELEVATOR_CANBUS);
    followerTalon =
        new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID, ElevatorConstants.ELEVATOR_CANBUS);
    followerTalon.setControl(new Follower(primaryTalon.getDeviceID(), MotorAlignmentValue.Opposed));

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = ElevatorConstants.kG;
    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kA = ElevatorConstants.kA;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.TORQUE_CURRENT_LIMIT;

    PhoenixUtil.tryUntilOk(5, () -> primaryTalon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> primaryTalon.setPosition(0));

    position = primaryTalon.getPosition();

    primaryVelocity = primaryTalon.getVelocity();
    primaryAppliedVolts = primaryTalon.getMotorVoltage();
    primaryTorqueCurrent = primaryTalon.getTorqueCurrent();
    primarySupplyCurrent = primaryTalon.getSupplyCurrent();
    primaryTemp = primaryTalon.getDeviceTemp();

    followerVelocity = followerTalon.getVelocity();
    followerAppliedVolts = followerTalon.getMotorVoltage();
    followerTorqueCurrent = followerTalon.getTorqueCurrent();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerTemp = followerTalon.getDeviceTemp();

    ParentDevice.optimizeBusUtilizationForAll(primaryTalon, followerTalon);

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                primaryVelocity,
                primaryAppliedVolts,
                primaryTorqueCurrent,
                primarySupplyCurrent,
                primaryTemp,
                followerVelocity,
                followerAppliedVolts,
                followerTorqueCurrent,
                followerSupplyCurrent,
                followerTemp));
  }

  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.primaryConnected =
        masterConnectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position,
                    primaryAppliedVolts,
                    primaryVelocity,
                    primarySupplyCurrent,
                    primaryTorqueCurrent)
                .isOK());

    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    followerAppliedVolts,
                    followerVelocity,
                    followerSupplyCurrent,
                    followerTorqueCurrent)
                .isOK());

    // inputs.position = position.getValue();
    // inputs.velocity = MetersPerSecond.of(primaryVelocity.getValue().in(RadiansPerSecond) *
    // ElevatorConstants.DRUM_RADIUS_METERS);
    // inputs.appliedVoltage = primaryAppliedVolts.getValue().getValue();
    // inputs.supplyCurrentAmps = primarySupplyCurrent.getValue().getValue();
  }

  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> primaryTalon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));
  }

  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> primaryTalon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));
  }

  private Angle linearToAngular(Distance linearPosition) {
    return Radians.of(linearPosition.in(Meters) / ElevatorConstants.DRUM_RADIUS_METERS);
  }

  private Distance angularToLinear(Angle angularPosition) {
    return Meters.of(angularPosition.in(Radians) * ElevatorConstants.DRUM_RADIUS_METERS);
  }
}
