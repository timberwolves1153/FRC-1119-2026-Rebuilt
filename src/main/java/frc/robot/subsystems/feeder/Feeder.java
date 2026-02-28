package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  public FeederIO feederIO;
  public FeederInputsAutoLogged feederInputs;

  private final double FEED_SPEED = 12; // Falcon wheels that go up

  public Feeder(FeederIO indexerIO) {
    this.feederIO = indexerIO;
    this.feederInputs = new FeederInputsAutoLogged();
  }

  public void setFeedVoltage(double volts) {
    feederIO.setFeederVoltage(volts);
  }

  public void stopFeed() {
    feederIO.stopFeeder();
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("feeder", feederInputs);
  }

  public Command feedCommand() {
    return startEnd(
        () -> {
          setFeedVoltage(FEED_SPEED);
        },
        () -> stopFeed());
  }
}
