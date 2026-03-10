package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFx implements FeederIO {

  private TalonFX feederMotor = new TalonFX(52);

  private final StatusSignal<Voltage> feederAppliedVolts = feederMotor.getMotorVoltage();
  private final StatusSignal<Current> feederCurrentValue = feederMotor.getSupplyCurrent();

  public FeederIOTalonFx() {
    config();
  }

  public void config() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50, feederAppliedVolts, feederCurrentValue);

    feederMotor.optimizeBusUtilization();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }

  @Override
  public void stopFeeder() {
    feederMotor.setVoltage(0);
  }

  @Override
  public void updateInputs(FeederInputs inputs) {
    inputs.feederAppliedVolts = feederMotor.getMotorVoltage().getValue().in(Volts);
    inputs.feederCurrentAmps = feederMotor.getSupplyCurrent().getValue().in(Amps);
  }
}
