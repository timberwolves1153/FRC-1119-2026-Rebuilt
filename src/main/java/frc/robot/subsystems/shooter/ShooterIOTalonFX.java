package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {

  private TalonFX shooter1Motor = new TalonFX(53);
  private TalonFX shooter2Motor = new TalonFX(54);
  private TalonFX shooter3Motor = new TalonFX(55);

  private final StatusSignal<Current> shooter1CurrentValue = shooter1Motor.getSupplyCurrent();
  private final StatusSignal<Voltage> shooter1AppliedVolts = shooter1Motor.getMotorVoltage();
  private final StatusSignal<Angle> shooter1Position = shooter1Motor.getPosition();
  private final StatusSignal<Temperature> shooter1Temp = shooter1Motor.getDeviceTemp();

  private final StatusSignal<Current> shooter2CurrentValue = shooter2Motor.getSupplyCurrent();
  private final StatusSignal<Voltage> shooter2AppliedVolts = shooter2Motor.getMotorVoltage();
  private final StatusSignal<Angle> shooter2Position = shooter2Motor.getPosition();
  private final StatusSignal<Temperature> shooter2Temp = shooter2Motor.getDeviceTemp();

  private final StatusSignal<Current> shooter3CurrentValue = shooter3Motor.getSupplyCurrent();
  private final StatusSignal<Voltage> shooter3AppliedVolts = shooter3Motor.getMotorVoltage();
  private final StatusSignal<Angle> shooter3Position = shooter3Motor.getPosition();
  private final StatusSignal<Temperature> shooter3Temp = shooter3Motor.getDeviceTemp();

  public ShooterIOTalonFX() {
    config();
  }

  public void config() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooter1Motor.getConfigurator().apply(config);
    shooter2Motor.getConfigurator().apply(config);
    shooter3Motor.getConfigurator().apply(config);
    shooter2Motor.setControl(new Follower(53, MotorAlignmentValue.Opposed));
    shooter3Motor.setControl(new Follower(53, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        shooter1CurrentValue,
        shooter1AppliedVolts,
        shooter1Position,
        shooter1Temp,
        shooter2CurrentValue,
        shooter2AppliedVolts,
        shooter2Position,
        shooter2Temp,
        shooter3CurrentValue,
        shooter3AppliedVolts,
        shooter3Position,
        shooter3Temp);
    shooter1Motor.optimizeBusUtilization();
    shooter2Motor.optimizeBusUtilization();
    shooter3Motor.optimizeBusUtilization();
  }

  public void setShooterVoltage(double volts) {
    shooter1Motor.setVoltage(volts);
  }

  public void setShooter1Voltage(double volts) {
    shooter1Motor.setVoltage(volts);
  }

  public void setShooter2Voltage(double volts) {
    shooter2Motor.setVoltage(volts);
  }

  public void setShooter3Voltage(double volts) {
    shooter3Motor.setVoltage(volts);
  }
}
