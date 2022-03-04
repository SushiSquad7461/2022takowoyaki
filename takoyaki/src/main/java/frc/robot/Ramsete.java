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

import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Ramsete {
  private DifferentialDriveVoltageConstraint voltageConstraint;
  private Drivetrain drivetrain;
  private SimpleMotorFeedforward ramseteFF; 
  
  // path enums
  public enum RamsetePath {
    FARBALL_SHOOT("output/farball-shoot.wpilib.json"),
    MIDBALL_SHOOT("output/midball-shoot.wpilib.json"),
    MIDBALL_WALLBALL("output/midball-wallball.wpilib.json"),
    SHOOT_MIDBALL_1_REVERSE("output/shoot-midball-1-reverse.wpilib.json"),
    SHOOT_MIDBALL_2("output/shoot-midball-2.wpilib.json"),
    SHOOT_TARMAC_REVERSE("output/shoot-tarmac-reverse.wpilib.json"),
    SHOOT_TERMINAL_1_REVERSE("output/shoot-terminal-1-reverse.wpilib.json"),
    SHOOT_TERMINAL_2("output/shoot-terminal-2.wpilib.json"),
    TARMAC_FARBALL("output/tarmac-farball.wpilib.json"),
    TARMAC_MIDBALL("output/tarmac-midball.wpilib.json"),
    TARMAC_WALLBALL("output/tarmac-wallball.wpilib.json"),
    TERMINAL_SHOOT_1_REVERSE("output/terminal-shoot-1-reverse.wpilib.json"),
    TERMINAL_SHOOT_2("output/terminal-shoot-2.wpilib.json"),
    WALLBALL_SHOOT("output/wallball-shoot.wpilib.json"),
    IOTA_TERMINAL_SHOOT_1_REVERSE("output/iota-terminal-shoot-1-reverse.wpilib.json"),
    IOTA_TERMINAL_SHOOT_2("output/iota-terminal-shoot-2.wpilib.json"),
    ZETA_TERMINAL_SHOOT_1_REVERSE("output/zeta-terminal-shoot-1-reverse.wpilib.json"),
    ZETA_TERMINAL_SHOOT_2("output/zeta-terminal-shoot-2.wpilib.json"),
    GAMMA_SHOOT_MIDBALL_1_REVERSE("output/output/shoot-midball-1-reverse.wpilib.json"),
    GAMMA_SHOOT_MIDBALL_2("output/output/shoot-midball-2.wpilib.json"),
    GAMMA_WALLBALL_SHOOT("output/output/wallball-shoot.wpilib.json"),
    GAMMA_SHOOT_TERMINAL_1_REVERSE("output/output/shoot-terminal-1-reverse.wpilib.json"),
    GAMMA_SHOOT_TERMINAL_2("output/output/shoot-terminal-2.wpilib.json"),
    GAMMA_TERMINAL_SHOOT_1_REVERSE("output/output/terminal-shoot-1-reverse.wpilib.json"),
    GAMMA_TERMINAL_SHOOT_2("output/output/terminal-shoot-2.wpilib.json"),
    GAMMA_TERMINAL_SHOOT_STRAIGHT("output/output/terminal-shoot-straight.wpilib.json");

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

  public RamseteCommand createRamseteCommand(RamsetePath path) {
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
