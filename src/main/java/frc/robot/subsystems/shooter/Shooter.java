package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public ShooterIO shooterIO;
  public ShooterInputsAutoLogged shooterInputs;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    this.shooterInputs = new ShooterInputsAutoLogged();
  }

  public void setShooterVoltage(double volts) {
    shooterIO.setShooterVoltage(volts);
  }

  public void stopShooter() {
    shooterIO.setShooterVoltage(0);
  }

  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("shooter", shooterInputs);
  }
}
