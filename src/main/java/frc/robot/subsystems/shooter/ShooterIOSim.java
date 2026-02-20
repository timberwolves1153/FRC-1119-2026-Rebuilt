package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private DCMotorSim Shooter1Motor;
  private DCMotorSim Shooter2Motor;
  private DCMotorSim Shooter3Motor;

  public ShooterIOSim() {
    Shooter1Motor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getNeoVortex(1));
  }
}
