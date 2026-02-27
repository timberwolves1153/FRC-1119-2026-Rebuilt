package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.Intake.Position;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double deployAppliedVolts = 0;
    public double intakeAppliedVolts = 0;
    public double retractAppliedVolts = 0;

    public Position state = Position.STOWED;
    public boolean isHomed = false;
    public Angle intakeDeployDegrees = Degrees.of(0);
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setDeployVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}

  public default BooleanSupplier isDeployStalled() {
    return () -> true;
  }

  public default void setDeployMotorPosition(Angle angle) {}
  ;
}
