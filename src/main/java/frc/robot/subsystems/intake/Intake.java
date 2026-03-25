package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public IntakeIO intakeIO;
  public IntakeInputsAutoLogged intakeInputs;

  public enum Position {
    HOMED(8),
    STOWED(28.5),
    AGITATE(110),
    DEPLOYED(134);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle angle() {
      return Degrees.of(degrees);
    }
  }

  public static final double INTAKE_SPEED = -11;
  public static final double STOP_SPEED = 0;

  public static final double HOMING_SPEED = -0.5;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    this.intakeInputs = new IntakeInputsAutoLogged();
    SmartDashboard.putNumber("deployAngle", 110);
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
    intakeIO.setIntakeVoltage(STOP_SPEED);
  }

  public void setPosition(Position goal) {
    intakeInputs.state = goal;
  }

  public void setDeployMotorPosition(Angle angle) {
    intakeIO.setDeployMotorPosition(angle);
  }

  public void setIsHomed(boolean isHomed) {
    intakeInputs.isHomed = isHomed;
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          setPosition(Position.DEPLOYED);
          setIntakeVoltage(INTAKE_SPEED);
        },
        () -> stopIntake());
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> setIntakeVoltage(STOP_SPEED));
  }

  public Command deployCommand() {
    return startEnd(
        () -> {
          setPosition(Position.DEPLOYED);
        },
        () -> stopDeploy());
  }

  public Command retractCommand() {
    return startEnd(
        () -> {
          setPosition(Position.STOWED);
          setIntakeVoltage(STOP_SPEED);
        },
        () -> stopDeploy());
  }

  private Command setAgitatePositionCommand() {
    return run(() -> setPosition(Position.AGITATE));
  }

  public Command agitateCommand() {
    return runOnce(() -> setIntakeVoltage(INTAKE_SPEED))
        .andThen(setAgitatePositionCommand())
        .handleInterrupt(
            () -> {
              setPosition(Position.DEPLOYED);
              stopIntake();
            });
  }

  public Command homingCommand() {
    return startEnd(
        () -> {
          setPosition(Position.HOMED);
        },
        () -> stopDeploy());
  }

  public Command dashboardDeployCommand() {
    return defer(
        () ->
            run(
                () ->
                    setDeployMotorPosition(
                        Degrees.of(SmartDashboard.getNumber("deployAngle", 110)))));
  }

  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("intake", intakeInputs);
  }
}
