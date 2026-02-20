package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  public IndexerIO indexerIO;
  public IndexerInputsAutoLogged indexerInputs;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
    this.indexerInputs = new IndexerInputsAutoLogged();
  }

  public void setIndexVoltage(double volts) {
    indexerIO.setIndexVoltage(volts);
  }

  public void stopIndex() {
    indexerIO.setIndexVoltage(0);
  }

  public void setLoadVoltage(double volts) {
    indexerIO.setLoadVoltage(volts);
  }

  public void stopLoad() {
    indexerIO.setLoadVoltage(0);
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("indexer", indexerInputs);
  }
}
