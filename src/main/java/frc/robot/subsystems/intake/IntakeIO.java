package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.Intake.Position;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double deployAppliedVolts = 0;
    public double intakeAppliedVolts = 0;

    public double deployCurrentValue = 0;

    public Position state = Position.HOMED;
    public boolean isHomed = false;
    public double intakeDeployDegrees = 0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setDeployVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}

  public default void setDeployMotorPosition(Angle angle) {}
  ;
}
