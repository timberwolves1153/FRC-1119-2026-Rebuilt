package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.Position;

public class IntakeIOVortex implements IntakeIO {
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

  private static final double DEPLOY_STALL_CURRENT_AMPS = .89;
  private static final Angle DEPLOY_POSITION_TOLERANCE = Degrees.of(5);

  public IntakeIOVortex() {
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
                    .withStatorCurrentLimit(Amps.of(50))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
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
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true));
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

  private boolean isPositionWithinTolerance(Angle targetPosition) {
    final Angle currentPosition = deployMotor.getPosition().getValue();
    // final Angle targetPosition = deployMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, DEPLOY_POSITION_TOLERANCE);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.deployAppliedVolts = deployMotor.getMotorVoltage().getValue().in(Volts);
    inputs.deployCurrentValue = deployMotor.getSupplyCurrent().getValue().in(Amps);

    inputs.intakeAppliedVolts = intakeAppliedVolts.getValue().in(Volts);

    inputs.intakeDeployDegrees =
        Units.rotationsToDegrees(deployMotor.getPosition().getValueAsDouble());

    switch (inputs.state) {
      case DEPLOYED:
        deployMotor.setControl(deployMotionMagicRequest.withPosition(Position.DEPLOYED.angle()));
        break;
      case STOWED:
        deployMotor.setControl(deployMotionMagicRequest.withPosition(Position.STOWED.angle()));
        break;
      case HOMED:
        {
          if (inputs.isHomed) {
            inputs.state = Position.STOWED;
          } else {
            if (deployCurrentValue.getValue().in(Amps) > DEPLOY_STALL_CURRENT_AMPS) {
              setDeployMotorPosition(Position.HOMED.angle());
              inputs.isHomed = true;
            } else {
              setDeployVoltage(Intake.HOMING_SPEED);
            }
          }
          break;
        }
      case AGITATE:
        {
          if (isPositionWithinTolerance(
              Position.AGITATE.angle())) { // We're near the agitate position so go to Deployed
            deployMotor.setControl(
                deployMotionMagicRequest.withPosition(Position.DEPLOYED.angle()));
          } else if (isPositionWithinTolerance(
              Position.DEPLOYED.angle())) { // We're near the deployed position so go to Agitate
            deployMotor.setControl(deployMotionMagicRequest.withPosition(Position.AGITATE.angle()));
          }
          break;
        }
      default:
        break;
    }

    SmartDashboard.putNumber("Deploy Angle", deployMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setDeployMotorPosition(Angle angle) {
    deployMotor.setPosition(angle);
  }
}
