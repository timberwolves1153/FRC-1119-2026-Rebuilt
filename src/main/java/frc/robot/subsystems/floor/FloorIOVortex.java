package frc.robot.subsystems.floor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class FloorIOVortex implements FloorIO {

  private SparkFlex floorMotor = new SparkFlex(51, MotorType.kBrushless);
  private SparkFlexConfig config = new SparkFlexConfig();

  public FloorIOVortex() {
    config();
  }

  public void config() {
    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kCoast);
    floorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setFloorVoltage(double volts) {
    floorMotor.setVoltage(volts);
  }

  @Override
  public void stopFloor() {
    floorMotor.setVoltage(0);
  }

  @Override
  public void updateInputs(FloorInputs inputs) {
    inputs.floorAppliedVolts = floorMotor.getAppliedOutput() * floorMotor.getBusVoltage();
    inputs.floorCurrentAmps = floorMotor.getOutputCurrent();
  }
}
