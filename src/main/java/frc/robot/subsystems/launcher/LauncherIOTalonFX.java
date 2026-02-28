package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class LauncherIOTalonFX implements LauncherIO {

  private TalonFX leftMotor = new TalonFX(53);
  private TalonFX centerMotor = new TalonFX(54);
  private TalonFX rightMotor = new TalonFX(55);

  private final StatusSignal<Current> leftCurrentValue = leftMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> leftAppliedVolts = leftMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();

  private final StatusSignal<Current> centerCurrentValue = centerMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> centerAppliedVolts = centerMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> centerVelocity = centerMotor.getVelocity();

  private final StatusSignal<Current> rightCurrentValue = rightMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> rightAppliedVolts = rightMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();

  private static final double STOP_SPEED = 0.0;
  private static final double LAUNCH_SPEED = -9.0;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);

  public LauncherIOTalonFX() {
    config();
  }

  private void config() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / Constants.krakenX60FreeSpeed.in(RotationsPerSecond)));

    leftMotor.getConfigurator().apply(config);
    centerMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
    centerMotor.setControl(new Follower(53, MotorAlignmentValue.Opposed));
    rightMotor.setControl(new Follower(53, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftCurrentValue,
        leftAppliedVolts,
        leftVelocity,
        centerCurrentValue,
        centerAppliedVolts,
        centerVelocity,
        rightCurrentValue,
        rightAppliedVolts,
        rightVelocity);

    leftMotor.optimizeBusUtilization();
    centerMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();
  }

  public void setLauncherVoltage(double volts) {
    leftMotor.setControl(voltageRequest.withOutput(Volts.of(volts)));
  }

  public void setLauncherRPM(double rpm) {
    leftMotor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
  }

  @Override
  public void stopLauncher() {
    leftMotor.set(STOP_SPEED);
  }

  @Override
  public boolean isVelocityInTolerance() {
    boolean isInVelocityMode = leftMotor.getAppliedControl().equals(velocityRequest);
    AngularVelocity currentVelocity = leftVelocity.getValue();
    AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();

    return isInVelocityMode && currentVelocity.isNear(targetVelocity, VELOCITY_TOLERANCE);
  }

  @Override
  public void updateInputs(LauncherInputs inputs) {
    inputs.leftLauncherAppliedVolts = leftMotor.getMotorVoltage().getValue().in(Volts);
    inputs.leftLauncherCurrentAmps = leftMotor.getSupplyCurrent().getValue().in(Amps);
    inputs.leftLauncherVelovityRPM = leftMotor.getVelocity().getValue().in(RPM);

    inputs.centerLauncherAppliedVolts = centerAppliedVolts.getValue().in(Volts);
    inputs.centerLauncherCurrentAmps = centerCurrentValue.getValue().in(Amps);
    inputs.centerLauncherVelovityRPM = centerVelocity.getValue().in(RPM);

    inputs.rightLauncherAppliedVolts = rightAppliedVolts.getValue().in(Volts);
    inputs.rightLauncherCurrentAmps = rightCurrentValue.getValue().in(Amps);
    inputs.rightLauncherVelovityRPM = rightVelocity.getValue().in(RPM);
  }
}
