package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.Position;
import java.util.function.BooleanSupplier;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX deployMotor = new TalonFX(41);
  private TalonFX intakeMotor = new TalonFX(42);

  private final StatusSignal<Current> deployCurrentValue = deployMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> deployAppliedVolts = deployMotor.getMotorVoltage();
  private final StatusSignal<Angle> deployPosition = deployMotor.getPosition();
  private final StatusSignal<Temperature> deployTemp = deployMotor.getDeviceTemp();

  private final StatusSignal<Current> intakeCurrentValue = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> intakeAppliedVolts = intakeMotor.getMotorVoltage();
  private final StatusSignal<Angle> intakePosition = intakeMotor.getPosition();
  private final StatusSignal<Temperature> intakeTemp = intakeMotor.getDeviceTemp();

  private static final double deployMotorReduction = 18.67;
  private static final AngularVelocity maxDeploySpeed =
      Constants.krakenX60FreeSpeed.div(deployMotorReduction);
  private final MotionMagicVoltage deployMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final double DEPLOY_STALL_CURRENT_AMPS = 6;

  public IntakeIOTalonFX() {
    configIntakeMotor();
    configureDeployMotor();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        deployCurrentValue,
        deployAppliedVolts,
        deployPosition,
        deployTemp,
        intakeCurrentValue,
        intakeAppliedVolts,
        intakePosition,
        intakeTemp);

    intakeMotor.optimizeBusUtilization();
    deployMotor.optimizeBusUtilization();
  }

  private void configureDeployMotor() {
    final TalonFXConfiguration deployConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(deployMotorReduction))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxDeploySpeed)
                    .withMotionMagicAcceleration(maxDeploySpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(
                        12.0
                            / maxDeploySpeed.in(
                                RotationsPerSecond)) // 12 volts when requesting max RPS
                );
    deployMotor.getConfigurator().apply(deployConfig);
  }

  public void configIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(config);
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotor.setVoltage(volts);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.deployAppliedVolts = deployAppliedVolts.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeDeployDegrees = deployMotor.getPosition().getValue();

    switch (inputs.state) {
      case DEPLOYED:
        deployMotor.setControl(deployMotionMagicRequest.withPosition(Position.DEPLOYED.angle()));
        break;
      case STOWED:
        deployMotor.setControl(deployMotionMagicRequest.withPosition(Position.STOWED.angle()));
        break;
      default:
        break;
    }
  }

  @Override
  public BooleanSupplier isDeployStalled() {
    return () -> deployCurrentValue.getValue().in(Amps) > DEPLOY_STALL_CURRENT_AMPS;
  }

  @Override
  public void setDeployMotorPosition(Angle angle) {
    deployMotor.setPosition(angle);
  }
}
