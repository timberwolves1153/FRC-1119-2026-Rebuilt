package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX deployMotor = new TalonFX(41);
  private TalonFX intakeMotor = new TalonFX(42);

  private final StatusSignal<Current> deployCurrentValue = deployMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> deployAppliedVolts = deployMotor.getMotorVoltage();
  private final StatusSignal<Angle> deployPosition = deployMotor.getPosition();
  private final StatusSignal<Temperature> deployTemp = deployMotor.getDeviceTemp();

  private final StatusSignal<Current> intakeCurrentValue = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> inatekAppliedVolts = intakeMotor.getMotorVoltage();
  private final StatusSignal<Angle> intakePosition = intakeMotor.getPosition();
  private final StatusSignal<Temperature> intakeTemp = intakeMotor.getDeviceTemp();

  public IntakeIOTalonFX() {
    config();
  }

  public void config() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployMotor.getConfigurator().apply(config);
    intakeMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        deployCurrentValue,
        deployAppliedVolts,
        deployPosition,
        deployTemp,
        intakeCurrentValue,
        inatekAppliedVolts,
        intakePosition,
        intakeTemp);
    deployMotor.optimizeBusUtilization();
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotor.setVoltage(volts);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }
}
