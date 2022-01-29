package frc.robot;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutoCommandSelector {
  private final Drivetrain drivetrain;
  private final Ramsete ramsete;
  private final Intake intake;

  // sequential command groups
  public final SequentialCommandGroup twoBallWall;
  public final SequentialCommandGroup twoBallMid;
  public final SequentialCommandGroup twoBallFar;
  public final SequentialCommandGroup threeBall;
  public final SequentialCommandGroup fiveBall;

  public final RamsetePath[] twoBallWallPaths = { RamsetePath.TARMAC_WALLBALL,
                                                  RamsetePath.WALLBALL_SHOOT };
  public final RamsetePath[] twoBallMidPaths = { RamsetePath.TARMAC_MIDBALL,
                                                 RamsetePath.MIDBALL_SHOOT }; 
  public final RamsetePath[] twoBallFarPaths = { RamsetePath.TARMAC_FARBALL,
                                                 RamsetePath.FARBALL_SHOOT };
  public final RamsetePath[] threeBallPaths = { RamsetePath.SHOOT_MIDBALL_1_REVERSE,
                                                RamsetePath.SHOOT_MIDBALL_2,
                                                RamsetePath.MIDBALL_WALLBALL,
                                                RamsetePath.WALLBALL_SHOOT };
  public final RamsetePath[] fiveBallPaths = { RamsetePath.SHOOT_MIDBALL_1_REVERSE,
                                               RamsetePath.SHOOT_MIDBALL_2,
                                               RamsetePath.MIDBALL_WALLBALL,
                                               RamsetePath.WALLBALL_SHOOT,
                                               RamsetePath.SHOOT_TERMINAL_1_REVERSE,
                                               RamsetePath.SHOOT_TERMINAL_2,
                                               RamsetePath.TERMINAL_SHOOT_1_REVERSE,
                                               RamsetePath.TERMINAL_SHOOT_2  };

  public final Map<SequentialCommandGroup, RamsetePath[]> pathArrayMap;

  public AutoCommandSelector(Drivetrain drivetrain, Ramsete ramsete, Intake intake) {
    // instantiate dependencies
    this.drivetrain = drivetrain;
    this.ramsete = ramsete;
    this.intake = intake;

    this.pathArrayMap = new HashMap<SequentialCommandGroup, RamsetePath[]>();

    // create command groups
    twoBallWall = new SequentialCommandGroup(
      new InstantCommand(intake::actuateIntake, intake),
      ramsete.createRamseteCommand(RamsetePath.TARMAC_WALLBALL),
      ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT));
    twoBallMid = new SequentialCommandGroup(
      new InstantCommand(intake::actuateIntake, intake),
      ramsete.createRamseteCommand(RamsetePath.TARMAC_MIDBALL),
      ramsete.createRamseteCommand(RamsetePath.MIDBALL_SHOOT));
    twoBallFar = new SequentialCommandGroup(
      new InstantCommand(intake::actuateIntake, intake),
      ramsete.createRamseteCommand(RamsetePath.TARMAC_FARBALL),
      ramsete.createRamseteCommand(RamsetePath.FARBALL_SHOOT));
    threeBall = new SequentialCommandGroup(
      ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_1_REVERSE),
      ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_2),
      ramsete.createRamseteCommand(RamsetePath.MIDBALL_WALLBALL),
      ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT));
    fiveBall = new SequentialCommandGroup(
      ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_1_REVERSE),
      ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_2),
      ramsete.createRamseteCommand(RamsetePath.MIDBALL_WALLBALL),
      ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT),
      ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_1_REVERSE),
      ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_2),
      ramsete.createRamseteCommand(RamsetePath.TERMINAL_SHOOT_1_REVERSE),
      ramsete.createRamseteCommand(RamsetePath.TERMINAL_SHOOT_2));
      
    
    // trajectory map
    pathArrayMap.put(twoBallWall, twoBallWallPaths);
    pathArrayMap.put(twoBallMid, twoBallMidPaths);
    pathArrayMap.put(twoBallFar, twoBallFarPaths);
    pathArrayMap.put(threeBall, threeBallPaths);
    pathArrayMap.put(fiveBall, fiveBallPaths);
  }

  public void setInitialDrivePose(SequentialCommandGroup auto) {
    if(pathArrayMap.containsKey(auto)) {
      drivetrain.setOdometry(pathArrayMap.get(auto)[0].getTrajectory());
    }  
  }
}