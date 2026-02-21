package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interpolation.LauncherTable;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {

  private Shooter launcher;
  private Indexer indexer;
  private Supplier<Double> distanceSupplier;
  private LauncherTable launcherTable;

  private SuperstructureState state = SuperstructureState.OFF;

  private Timer timer = new Timer();

  public enum SuperstructureState {
    OFF,
    PASSING,
    SCORING
  }

  public Superstructure(
      Shooter launcher,
      Indexer indexer,
      Supplier<Double> distanceSupplier,
      LauncherTable launcherTable) {
    this.launcher = launcher;
    this.indexer = indexer;
    this.distanceSupplier = distanceSupplier;
    launcherTable = new LauncherTable();

    SmartDashboard.putNumber("Index Speed", -10);
    SmartDashboard.putNumber("Feeder Speed", 10);
    SmartDashboard.putNumber("Launcher Speed", -9);
  }

  public void scoreBalls() {
    indexer.setIndexVoltage(SmartDashboard.getNumber("Index Speed", -10));
    indexer.setLoadVoltage(SmartDashboard.getNumber("Feeder Speed", 12));
    launcher.setShooterVoltage(SmartDashboard.getNumber("Launcher Speed", -9));
    // launcher.setShooterVoltage(
    //     launcherTable.getLaunchVoltageSupplier(distanceSupplier.get()).getAsDouble());
  }

  public void stopEverything() {
    indexer.stopIndex();
    indexer.stopLoad();
    launcher.stopShooter();
  }

  public void setState(SuperstructureState newState) {
    this.state = newState;
  }

  @Override
  public void periodic() {
    switch (this.state) {
      case SCORING:
        {
          scoreBalls();
          break;
        }
      case PASSING:
        {
          stopEverything();
          break;
        }
      case OFF:
        {
          stopEverything();
          break;
        }
      default:
        {
          stopEverything();
          break;
        }
    }
  }
}
