package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drivetrain;

public class AutoCommandSelector {
  private final Drivetrain Drivetrain;
  private final Ramsete ramsete;

  // sequential command groups
  public final SequentialCommandGroup test;
  public final SequentialCommandGroup forward;
  public final SequentialCommandGroup curve;

  public final RamsetePath[] testPath = { Ramsete.RamsetePath.FORWARD, Ramsete.RamsetePath.CURVE };
  public final RamsetePath[] forwardPath = { Ramsete.RamsetePath.FORWARD }; 
  public final RamsetePath[] curvePath = { Ramsete.RamsetePath.CURVE };

  public final Map<SequentialCommandGroup, RamsetePath[]> firstTrajectoryMap;

  public AutoCommandSelector(Drivetrain Drivetrain, Ramsete ramsete) {
    // instantiate dependencies
    this.Drivetrain = Drivetrain;
    this.ramsete = ramsete;

    this.firstTrajectoryMap = new HashMap<SequentialCommandGroup, RamsetePath[]>();

    // create command groups
    test = new SequentialCommandGroup(
      ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD),
      ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
    forward = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD));
    curve = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
    
    // trajectory map
    firstTrajectoryMap.put(test, testPath);
    firstTrajectoryMap.put(forward, forwardPath);
    firstTrajectoryMap.put(curve, curvePath);
  }

  public void setInitialDrivePose(SequentialCommandGroup auto) {
    if(firstTrajectoryMap.containsKey(auto)) {
      Drivetrain.setOdometry(firstTrajectoryMap.get(auto)[0].getTrajectory());
    }  
  }
}