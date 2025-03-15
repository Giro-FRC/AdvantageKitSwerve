package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class GoToPath extends Command {
  private final Drive drive;
  private CommandScheduler scheduler;
  // private final List<Waypoint> waypoints;
  private final Pose2d targetPose;

  public GoToPath(Drive drive, CommandScheduler scheduler, Pose2d targetPose) {
    this.drive = drive;
    this.scheduler = scheduler;
    // this.waypoints = waypoints;
    this.targetPose = targetPose;
    addRequirements(drive);
  }

  @Override
  public void initialize() {

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    Logger.recordOutput("GoToPath/RobotPose", drive.getPose());
    Logger.recordOutput("GoToPath/TargetPose", targetPose);

    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    scheduler.schedule(pathfindingCommand);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
