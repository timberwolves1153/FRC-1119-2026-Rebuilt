package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOVortex implements IndexerIO {

  private SparkFlex indexMotor = new SparkFlex(51, MotorType.kBrushless);
  private TalonFX loadMotor = new TalonFX(52);
  private SparkFlexConfig config = new SparkFlexConfig();

  private final StatusSignal<Current> loadCurrentValue = loadMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> loadAppliedVolts = loadMotor.getMotorVoltage();
  private final StatusSignal<Angle> loadPosition = loadMotor.getPosition();
  private final StatusSignal<Temperature> loadTemp = loadMotor.getDeviceTemp();

  public IndexerIOVortex() {
    config();
  }

  public void config() {
    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kCoast);
    indexMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    loadMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, loadCurrentValue, loadAppliedVolts, loadPosition, loadTemp);

    loadMotor.optimizeBusUtilization();
  }

  @Override
  public void setIndexVoltage(double volts) {
    indexMotor.setVoltage(volts);
  }

  @Override
  public void setLoadVoltage(double volts) {
    loadMotor.setVoltage(volts);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.IndexAppliedVolts = indexMotor.getAppliedOutput() * 12;
  }
}
