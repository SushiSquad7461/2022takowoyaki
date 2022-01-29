package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drivetrain;

public class AutoCommandSelector {
  private final Drivetrain drivetrain;
  private final Ramsete ramsete;

  // sequential command groups
  public final SequentialCommandGroup test;
  public final SequentialCommandGroup forward;
  public final SequentialCommandGroup curve;

  public final RamsetePath[] testPath = { Ramsete.RamsetePath.FORWARD, Ramsete.RamsetePath.CURVE };
  public final RamsetePath[] forwardPath = { Ramsete.RamsetePath.FORWARD }; 
  public final RamsetePath[] curvePath = { Ramsete.RamsetePath.CURVE };

  public final Map<SequentialCommandGroup, RamsetePath[]> pathArrayMap;

  public AutoCommandSelector(Drivetrain drivetrain, Ramsete ramsete) {
    // instantiate dependencies
    this.drivetrain = drivetrain;
    this.ramsete = ramsete;

    this.pathArrayMap = new HashMap<SequentialCommandGroup, RamsetePath[]>();

    // create command groups
    test = new SequentialCommandGroup(
      ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD),
      ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
    forward = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD));
    curve = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
    
    // trajectory map
    pathArrayMap.put(test, testPath);
    pathArrayMap.put(forward, forwardPath);
    pathArrayMap.put(curve, curvePath);
  }

  public void setInitialDrivePose(SequentialCommandGroup auto) {
    if(pathArrayMap.containsKey(auto)) {
      drivetrain.setOdometry(pathArrayMap.get(auto)[0].getTrajectory());
    }  
  }
}