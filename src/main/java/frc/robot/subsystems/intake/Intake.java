package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public IntakeIO intakeIO;
  public IntakeInputsAutoLogged intakeInputs;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    this.intakeInputs = new IntakeInputsAutoLogged();
  }

  public void setDeployVoltage(double volts) {
    intakeIO.setDeployVoltage(volts);
  }

  public void setIntakeVoltage(double volts) {
    intakeIO.setIntakeVoltage(volts);
  }

  public void stopDeploy() {
    intakeIO.setDeployVoltage(0);
  }

  public void stopIntake() {
    intakeIO.setIntakeVoltage(0);
  }

  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("intake", intakeInputs);
  }
}
