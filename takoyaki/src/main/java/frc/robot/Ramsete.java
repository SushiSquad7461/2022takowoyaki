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
  private DifferentialDriveVoltageConstraint voltageConstraint;
  private Drivetrain drivetrain;
  private SimpleMotorFeedforward ramseteFF; 

  public enum PathPlannerPath {
    SHOOT_MIDBALL(PathPlanner.loadPath("shoot-midball", 2, 2, true)),
    MIDBALL_WALLBALL(PathPlanner.loadPath("midball-wallball", 2, 2, false)),
    WALLBALL_SHOOT(PathPlanner.loadPath("wallball-shoot", 2, 2, false)),
    SHOOT_TERMINAL(PathPlanner.loadPath("shoot-terminal", 2, 2, true)),
    TERMINAL_SHOOT(PathPlanner.loadPath("terminal-shoot", 2, 2, false)),
    TWO_BALL_FAR(PathPlanner.loadPath("two-ball-far", 2, 2, false)),
    ONE_BALL_FAR_MID(PathPlanner.loadPath("one-ball-far-mid", 2, 2, false)),
    ONE_BALL_FAR_FAR(PathPlanner.loadPath("one-ball-far-far", 2, 2, false));

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

    voltageConstraint = new DifferentialDriveVoltageConstraint(
      ramseteFF,
      Constants.kDrive.DRIVE_KINEMATICS,
      Constants.kDrive.MAX_VOLTAGE);
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
      //.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  public DifferentialDriveVoltageConstraint getVoltageConstraint() {
    return voltageConstraint;
  }

}
