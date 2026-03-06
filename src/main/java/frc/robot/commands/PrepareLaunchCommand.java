package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.launcher.Launcher;
import java.util.Optional;
import java.util.function.Supplier;

public class PrepareLaunchCommand extends Command {
  private static final InterpolatingTreeMap<Distance, Double> distanceToRPMMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              Interpolator.forDouble().interpolate(startValue, endValue, t));

  static {
    // distanceToRPMMap.put(Inches.of(52.0), 2800.0);
    // distanceToRPMMap.put(Inches.of(114.4), 3275.0);
    // distanceToRPMMap.put(Inches.of(165.5), 3650.0);
    distanceToRPMMap.put(Meters.of(2.76), -2800.0);
    distanceToRPMMap.put(Meters.of(3.539), -3150.0);
    distanceToRPMMap.put(Inches.of(196), -3500.0);
  }

  private final Launcher launcher;
  private final Supplier<Pose2d> robotPoseSupplier;

  public PrepareLaunchCommand(Launcher launcher, Supplier<Pose2d> robotPoseSupplier) {
    this.launcher = launcher;
    this.robotPoseSupplier = robotPoseSupplier;
    addRequirements(launcher);
  }

  public boolean isReadyToLaunch() {
    return launcher.isVelocityInTolerance();
  }

  private Distance getDistanceToHub() {
    Translation2d hubTranslation2d;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      hubTranslation2d = FieldConstants.Hub.blueHubCenter.getTranslation();
    } else {
      hubTranslation2d = FieldConstants.Hub.redHubCenter.getTranslation();
    }
    Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
    return Meters.of(robotPosition.getDistance(hubTranslation2d));
  }

  @Override
  public void execute() {
    Distance distanceToHub = getDistanceToHub();
    double launchRPM = distanceToRPMMap.get(distanceToHub);
    launcher.setLauncherRPM(launchRPM);
    SmartDashboard.putNumber("Distance to HUb (inches)", distanceToHub.in(Inches));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stopLauncher();
  }
}
