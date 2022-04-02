package frc.robot;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Ramsete {
  private Drivetrain drivetrain;
  private SimpleMotorFeedforward ramseteFF;

  public enum PathPlannerPath {
    // SHOOT_MIDBALL(PathPlanner.loadPath("shoot-midball", 2, 2, true)),
    // MIDBALL_WALLBALL(PathPlanner.loadPath("midball-wallball", 2, 2, false)),
    // WALLBALL_SHOOT(PathPlanner.loadPath("wallball-shoot", 2, 2, false)),
    // SHOOT_TERMINAL(PathPlanner.loadPath("shoot-terminal", 2, 2, false)),
    // TERMINAL_SHOOT(PathPlanner.loadPath("terminal-shoot", 2, 2, false)),
    SHOOT_MIDBALL(PathPlanner.loadPath("shoot-midball", 8, 3.5, true)),
    MIDBALL_WALLBALL(PathPlanner.loadPath("midball-wallball", 8, 2.5, false)),
    MIDBALL_SHOOT(PathPlanner.loadPath("midball-shoot", 8, 3, false)),
    WALLBALL_SHOOT(PathPlanner.loadPath("wallball-shoot", 7, 4, false)),
    SHOOT_TERMINAL(PathPlanner.loadPath("shoot-terminal", 6, 3, false)),
    SHOOT_WALL(PathPlanner.loadPath("shoot-wall", 6, 4, true)),
    WALL_TERMINAL(PathPlanner.loadPath("wall-terminal", 7, 2.5, false)),
    TERMINAL_WALL(PathPlanner.loadPath("terminal-wall", 5, 3, true)),
    WALL_SHOOT(PathPlanner.loadPath("wall-shoot", 6, 3, false)),
    TERMINAL_SHOOT(PathPlanner.loadPath("terminal-shoot", 7, 3, false)),
    TWO_BALL_FAR(PathPlanner.loadPath("two-ball-far", 3, 2, false)),
    ONE_BALL_FAR_MID(PathPlanner.loadPath("one-ball-far-mid", 2, 2, false)),
    ONE_BALL_FAR_FAR(PathPlanner.loadPath("one-ball-far-far", 2, 2, false)),
    FAR_DEFENSE(PathPlanner.loadPath("far-defense", 3, 2, true));

    private PathPlannerTrajectory traj;

    private PathPlannerPath(PathPlannerTrajectory traj) {
      this.traj = traj;
    }

    public PathPlannerTrajectory getTrajectory() {
      return traj;
    }
  }

  public Ramsete(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    ramseteFF = new SimpleMotorFeedforward(
        Constants.kDrive.ksVolts,
        Constants.kDrive.kvVoltSecondsPerMeter,
        Constants.kDrive.kaVoltSecondsSquaredPerMeter);
  }

  public RamseteCommand createRamseteCommand(PathPlannerPath path) {
    return new RamseteCommand(
        path.getTrajectory(),
        drivetrain::getPose,
        new RamseteController(
            Constants.kDrive.RAMSETE_B,
            Constants.kDrive.RAMSETE_ZETA),
        ramseteFF,
        Constants.kDrive.DRIVE_KINEMATICS,
        drivetrain::getWheelSpeeds,
        new PIDController(
            Constants.kDrive.kPDriveVel,
            Constants.kDrive.kIDrive,
            Constants.kDrive.kDDrive),
        new PIDController(
            Constants.kDrive.kPDriveVel,
            Constants.kDrive.kIDrive,
            Constants.kDrive.kDDrive),
        drivetrain::tankDriveVolts,
        drivetrain);
  }

}
