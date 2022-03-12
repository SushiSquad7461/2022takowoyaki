package frc.robot;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoCommandSelector {
    private final Drivetrain drivetrain;
    private final Ramsete ramsete;
    private final Intake intake;
    private final Shooter shooter;
    private final Hopper hopper;

    // sequential command groups
    public final SequentialCommandGroup twoBallWall;
    public final SequentialCommandGroup twoBallMid;
    public final SequentialCommandGroup twoBallFar;
    public final SequentialCommandGroup threeBall;
    public final SequentialCommandGroup fiveBall;
    public final SequentialCommandGroup reverseSpline;
    public final SequentialCommandGroup iotaFiveBall;
    public final SequentialCommandGroup zetaFiveBall;
    public final SequentialCommandGroup fourBall;

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
            // RamsetePath.MIDBALL_WALLBALL,
            RamsetePath.WALLBALL_SHOOT,
            RamsetePath.SHOOT_TERMINAL_1_REVERSE,
            RamsetePath.SHOOT_TERMINAL_2,
            RamsetePath.GAMMA_TERMINAL_SHOOT_STRAIGHT };
    // RamsetePath.TERMINAL_SHOOT_1_REVERSE,
    // RamsetePath.TERMINAL_SHOOT_2 };
    public final RamsetePath[] reverseSplinePaths = { RamsetePath.SHOOT_TARMAC_REVERSE };
    public final RamsetePath[] iotaFiveBallPaths = { RamsetePath.SHOOT_MIDBALL_1_REVERSE,
            RamsetePath.SHOOT_MIDBALL_2,
            // RamsetePath.MIDBALL_WALLBALL,
            RamsetePath.WALLBALL_SHOOT,
            RamsetePath.SHOOT_TERMINAL_1_REVERSE,
            RamsetePath.SHOOT_TERMINAL_2,
            RamsetePath.IOTA_TERMINAL_SHOOT_1_REVERSE,
            RamsetePath.IOTA_TERMINAL_SHOOT_2 };
    public final RamsetePath[] zetaFiveBallPaths = { RamsetePath.SHOOT_MIDBALL_1_REVERSE,
            RamsetePath.SHOOT_MIDBALL_2,
            // RamsetePath.MIDBALL_WALLBALL,
            RamsetePath.WALLBALL_SHOOT,
            RamsetePath.SHOOT_TERMINAL_1_REVERSE,
            RamsetePath.SHOOT_TERMINAL_2,
            RamsetePath.ZETA_TERMINAL_SHOOT_1_REVERSE,
            RamsetePath.ZETA_TERMINAL_SHOOT_2 };
    public final RamsetePath[] fourBallPaths = { RamsetePath.TARMAC_MIDBALL,
            RamsetePath.MIDBALL_SHOOT,
            RamsetePath.SHOOT_TERMINAL_1_REVERSE,
            RamsetePath.SHOOT_TERMINAL_2,
            RamsetePath.TERMINAL_SHOOT_1_REVERSE,
            RamsetePath.TERMINAL_SHOOT_2 };
    public final Map<SequentialCommandGroup, RamsetePath[]> pathArrayMap;

    private AutoShoot getAutoShoot() {
        return new AutoShoot(shooter, hopper, intake);
    }

    public AutoCommandSelector(Drivetrain drivetrain, Ramsete ramsete, Intake intake, Shooter shooter, Hopper hopper) {
        // instantiate dependencies
        this.drivetrain = drivetrain;
        this.ramsete = ramsete;
        this.intake = intake;
        this.shooter = shooter;
        this.hopper = hopper;

        // not sure if this should be this.shooter and this.hopper or just shooter,
        // hopper
        this.pathArrayMap = new HashMap<SequentialCommandGroup, RamsetePath[]>();

        // create command groups
        twoBallWall = new SequentialCommandGroup(
                new InstantCommand(intake::toggleIntake, intake),
                new WaitCommand(7),
                ramsete.createRamseteCommand(RamsetePath.TARMAC_WALLBALL),
                ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT),
                new InstantCommand(intake::toggleIntake, intake),
                getAutoShoot().withTimeout(5));

        twoBallMid = new SequentialCommandGroup(
                new InstantCommand(intake::toggleIntake, intake),
                ramsete.createRamseteCommand(RamsetePath.TARMAC_MIDBALL),
                new InstantCommand(intake::toggleIntake, intake),
                ramsete.createRamseteCommand(RamsetePath.MIDBALL_SHOOT),
                getAutoShoot());

        twoBallFar = new SequentialCommandGroup(
                new InstantCommand(intake::toggleIntake, intake),
                new WaitCommand(7),
                ramsete.createRamseteCommand(RamsetePath.TARMAC_FARBALL),
                new InstantCommand(intake::toggleIntake, intake),
                ramsete.createRamseteCommand(RamsetePath.FARBALL_SHOOT),
                getAutoShoot().withTimeout(5));

        threeBall = new SequentialCommandGroup(
                getAutoShoot().withTimeout(1),
                ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_1_REVERSE),
                new ParallelCommandGroup(new RunCommand(intake::toggleIntake, intake).withTimeout(0),
                        ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_2)),
                ramsete.createRamseteCommand(RamsetePath.MIDBALL_WALLBALL),
                new ParallelCommandGroup(new InstantCommand(shooter::setSetpoint, shooter),
                        new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT)),
                new ParallelCommandGroup(new RunCommand(shooter::runKicker, shooter).withTimeout(6),
                        new RunCommand(hopper::runHopper, hopper).withTimeout(6)),
                new ParallelCommandGroup(new InstantCommand(shooter::stopKicker, shooter),
                        new InstantCommand(hopper::stop, hopper)));

        fiveBall = new SequentialCommandGroup(
                // shoot first ball
                getAutoShoot().withTimeout(0.5),
                // complete path to first ball and actuate intake
                ramsete.createRamseteCommand(RamsetePath.GAMMA_SHOOT_MIDBALL_1_REVERSE),
                // run intake and pick up mid ball and wall ball
                new ParallelCommandGroup(new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.GAMMA_SHOOT_MIDBALL_2)),
                // drive to the hub to shoot
                ramsete.createRamseteCommand(RamsetePath.GAMMA_WALLBALL_SHOOT)
                        .andThen(new InstantCommand(intake::toggleIntake, intake)),
                // shoot for 1 second
                getAutoShoot().withTimeout(1),
                // new RunCommand(intake::toggleIntake, intake).withTimeout(1)),
                // stop shooting
                // drive halfway to the terminal
                // ramsete.createRamseteCommand(RamsetePath.GAMMA_SHOOT_TERMINAL_1_REVERSE),
                ramsete.createRamseteCommand(RamsetePath.GAMMA_SHOOT_TERMINAL_STRAIGHT_1),
                // run intake and drive to terminal balls
                new ParallelCommandGroup(new InstantCommand(intake::toggleIntake, intake),
                        // ramsete.createRamseteCommand(RamsetePath.GAMMA_SHOOT_TERMINAL_2)),
                        ramsete.createRamseteCommand(RamsetePath.GAMMA_SHOOT_TERMINAL_STRAIGHT_2)),
                new WaitCommand(0.5),
                new ParallelCommandGroup(new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.GAMMA_TERMINAL_SHOOT_STRAIGHT)),
                // drive halfway back to the hub
                // ramsete.createRamseteCommand(RamsetePath.TERMINAL_SHOOT_1_REVERSE),
                // // drive to hub and rev shooter
                // new
                // ParallelCommandGroup(ramsete.createRamseteCommand(RamsetePath.TERMINAL_SHOOT_2),
                // new InstantCommand(shooter::setSetpoint, shooter)),
                // shoot final two balls
                getAutoShoot());

        reverseSpline = new SequentialCommandGroup(
                ramsete.createRamseteCommand(RamsetePath.SHOOT_TARMAC_REVERSE));

        iotaFiveBall = new SequentialCommandGroup(
                getAutoShoot().withTimeout(10),
                ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_1_REVERSE),
                new ParallelCommandGroup(new RunCommand(intake::toggleIntake, intake).withTimeout(0),
                        ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_2)),
                new ParallelCommandGroup(new InstantCommand(shooter::setSetpoint, shooter),
                        new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT)),
                new ParallelCommandGroup(new RunCommand(shooter::runKicker, shooter).withTimeout(1),
                        new RunCommand(hopper::runHopper, hopper).withTimeout(1)),
                new ParallelCommandGroup(new InstantCommand(shooter::stopKicker, shooter),
                        new InstantCommand(hopper::stop, hopper)),
                new ParallelCommandGroup(ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_1_REVERSE),
                        new InstantCommand(() -> shooter.zeroSetpoint(), shooter)),
                new ParallelCommandGroup(new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_2))
                                .andThen(new InstantCommand(intake::toggleIntake, intake)),
                ramsete.createRamseteCommand(RamsetePath.IOTA_TERMINAL_SHOOT_1_REVERSE),
                ramsete.createRamseteCommand(RamsetePath.IOTA_TERMINAL_SHOOT_2),
                getAutoShoot());

        zetaFiveBall = new SequentialCommandGroup(
                getAutoShoot().withTimeout(1),
                ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_1_REVERSE),
                new ParallelCommandGroup(new RunCommand(intake::toggleIntake, intake).withTimeout(0),
                        ramsete.createRamseteCommand(RamsetePath.SHOOT_MIDBALL_2)),
                new ParallelCommandGroup(new InstantCommand(shooter::setSetpoint, shooter),
                        new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.WALLBALL_SHOOT)),
                new ParallelCommandGroup(new RunCommand(shooter::runKicker, shooter).withTimeout(1),
                        new RunCommand(hopper::runHopper, hopper).withTimeout(1)),
                new ParallelCommandGroup(new InstantCommand(shooter::stopKicker, shooter),
                        new InstantCommand(hopper::stop, hopper)),
                new ParallelCommandGroup(ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_1_REVERSE),
                        new InstantCommand(() -> shooter.zeroSetpoint(), shooter)),
                new ParallelCommandGroup(new InstantCommand(intake::toggleIntake, intake),
                        ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_2))
                                .andThen(new InstantCommand(intake::toggleIntake, intake)),
                ramsete.createRamseteCommand(RamsetePath.ZETA_TERMINAL_SHOOT_1_REVERSE),
                ramsete.createRamseteCommand(RamsetePath.ZETA_TERMINAL_SHOOT_2),
                getAutoShoot());

        fourBall = new SequentialCommandGroup(
                new InstantCommand(intake::toggleIntake, intake),
                ramsete.createRamseteCommand(RamsetePath.TARMAC_MIDBALL),
                new InstantCommand(intake::toggleIntake, intake),
                ramsete.createRamseteCommand(RamsetePath.MIDBALL_SHOOT),
                getAutoShoot(),
                ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_1_REVERSE),
                new ParallelCommandGroup(
                        ramsete.createRamseteCommand(RamsetePath.SHOOT_TERMINAL_2),
                        new RunCommand(intake::toggleIntake, intake)),
                new ParallelCommandGroup(
                        ramsete.createRamseteCommand(RamsetePath.TERMINAL_SHOOT_1_REVERSE),
                        new RunCommand(intake::toggleIntake, intake)),
                ramsete.createRamseteCommand(RamsetePath.TERMINAL_SHOOT_2),
                getAutoShoot());

        // trajectory map
        pathArrayMap.put(twoBallWall, twoBallWallPaths);
        pathArrayMap.put(twoBallMid, twoBallMidPaths);
        pathArrayMap.put(twoBallFar, twoBallFarPaths);
        pathArrayMap.put(threeBall, threeBallPaths);
        pathArrayMap.put(fiveBall, fiveBallPaths);
        pathArrayMap.put(reverseSpline, reverseSplinePaths);
        pathArrayMap.put(iotaFiveBall, iotaFiveBallPaths);
        pathArrayMap.put(zetaFiveBall, zetaFiveBallPaths);
        pathArrayMap.put(fourBall, fourBallPaths);
    }

    public void setInitialDrivePose(SequentialCommandGroup auto) {
        if (pathArrayMap.containsKey(auto)) {
            drivetrain.setOdometry(pathArrayMap.get(auto)[0].getTrajectory());
        }
    }
}