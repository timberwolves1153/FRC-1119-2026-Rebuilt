package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public IntakeIO intakeIO;
  public IntakeInputsAutoLogged intakeInputs;
  public boolean isHomed = false;

  public enum Position {
    HOMED(110), // DETERMINE THIS ANGLE
    STOWED(100), // DETERMINE THIS ANGLE
    DEPLOYED(-4); // DETERMINE THIS ANGLE

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

  public static final double HOMING_SPEED = 0.1;

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
    intakeIO.setIntakeVoltage(STOP_SPEED);
  }

  public void setPosition(Position goal) {
    intakeInputs.state = goal;
  }

  public void setDeployMotorPosition(Angle angle) {
    intakeIO.setDeployMotorPosition(angle);
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          // setPosition(Position.DEPLOYED);
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
        },
        () -> stopDeploy());
  }

  public Command homingCommand() {
    return Commands.sequence(
            runOnce(() -> setDeployVoltage(HOMING_SPEED)),
            Commands.waitUntil(intakeIO.isDeployStalled()),
            runOnce(
                () -> {
                  intakeIO.setDeployMotorPosition(Position.HOMED.angle());
                  isHomed = true;
                  setPosition(Position.STOWED);
                }))
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("intake", intakeInputs);
  }
}
