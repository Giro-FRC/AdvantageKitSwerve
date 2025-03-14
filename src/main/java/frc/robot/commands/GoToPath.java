package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GoToPath extends Command {
  private final Drive drive;
  private CommandScheduler scheduler;
  private final Pose2d targetPose;

  private final boolean flipping;

  public GoToPath(Drive drive, CommandScheduler scheduler, Pose2d targetPose, boolean flipping) {
    this.drive = drive;
    this.scheduler = scheduler;
    this.targetPose = targetPose;
    this.flipping = flipping;
    addRequirements(drive);
  }

  @Override
  public void initialize() {

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    Logger.recordOutput("GoToPath/RobotPose", drive.getPose());
    Logger.recordOutput("GoToPath/TargetPose", targetPose);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(targetPose, targetPose);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));

    path.preventFlipping = flipping;

    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

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
