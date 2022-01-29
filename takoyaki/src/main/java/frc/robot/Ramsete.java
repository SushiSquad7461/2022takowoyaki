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

import frc.robot.subsystems.Drivetrain;

public class Ramsete {
  private DifferentialDriveVoltageConstraint voltageConstraint;
  private Drivetrain drivetrain;
  private SimpleMotorFeedforward ramseteFF; 
  
  // path enums
  public enum RamsetePath {
    FORWARD("paths/output/forward.wpilib.json"),
    CURVE("paths/output/curve.wpilib.json");

    private String json;
    RamsetePath(String json) {
      this.json = json;
    }

    public Trajectory getTrajectory() {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
        return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        return new Trajectory(); 
      }
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

  public SequentialCommandGroup createRamseteCommand(RamsetePath path) {
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
      drivetrain)
      .andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  public DifferentialDriveVoltageConstraint getVoltageConstraint() {
    return voltageConstraint;
  }

}
