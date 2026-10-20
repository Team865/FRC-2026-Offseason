package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class superstructure extends SubsystemBase {
  public static enum superState {
    HOME,
    INTAKE_CORAL,
    ARM_CORAL,
    PRE_L1_VERTICAL,
    PRE_L1_HORIZONTAL,
    PRE_L2,
    PRE_L3,
    PRE_L4,
    SCORE_CORAL_L1_VERTICAL,
    SCORE_CORAL_L1_HORIZONTAL,
    SCORE_CORAL_L2,
    SCORE_CORAL_L3,
    SCORE_CORAL_L4,
    PURGE_CORAL,
    PRE_ALGAE_HIGH,
    PRE_ALGAE_LOW,
    PRE_ALGAE_GROUND,
    PRE_ALGAE_LOLLIPOP,
    INTAKE_ALGAE,
    ALGAE_HOME,
    PURGE_ALGAE,
    SCORE_BARGE,
    SCORE_PROCESSOR
  }

  public superstructure() {}
}
