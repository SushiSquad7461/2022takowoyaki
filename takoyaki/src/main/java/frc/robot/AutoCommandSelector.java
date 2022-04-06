package frc.robot;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Ramsete.PathPlannerPath;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class AutoCommandSelector {
        private final Drivetrain drivetrain;
        private final Ramsete ramsete;
        private final Intake intake;
        private final Shooter shooter;
        private final Hopper hopper;

        // sequential command groups
        public final SequentialCommandGroup fiveBall;
        public final SequentialCommandGroup twoBallFar;
        public final SequentialCommandGroup oneBallFarMid;
        public final SequentialCommandGroup oneBallFarFar;

        public final PathPlannerPath[] fiveBallPaths = {
                        PathPlannerPath.SHOOT_MIDBALL,
                        PathPlannerPath.MIDBALL_WALLBALL,
                        PathPlannerPath.WALLBALL_SHOOT,
                        PathPlannerPath.SHOOT_TERMINAL,
                        PathPlannerPath.TERMINAL_SHOOT
        };

        public final PathPlannerPath[] twoBallFarPaths = {
                        PathPlannerPath.TWO_BALL_FAR,
                        PathPlannerPath.FAR_DEFENSE
        };

        public final PathPlannerPath[] oneBallFarMidPaths = {
                        PathPlannerPath.ONE_BALL_FAR_MID
        };

        public final PathPlannerPath[] oneBallFarFarPaths = {
                        PathPlannerPath.ONE_BALL_FAR_FAR
        };

        public final Map<SequentialCommandGroup, PathPlannerPath[]> pathArrayMap;

        private AutoShoot getAutoShoot() {
                return new AutoShoot(shooter, hopper, intake, ShooterState.FENDER);
        }

        private AutoShoot getRangedAutoShoot() {
                return new AutoShoot(shooter, hopper, intake, ShooterState.AUTO);
        }

        public AutoCommandSelector(Drivetrain drivetrain, Ramsete ramsete, Intake intake, Shooter shooter,
                        Hopper hopper) {
                // instantiate dependencies
                this.drivetrain = drivetrain;
                this.ramsete = ramsete;
                this.intake = intake;
                this.shooter = shooter;
                this.hopper = hopper;

                this.pathArrayMap = new HashMap<SequentialCommandGroup, PathPlannerPath[]>();

                fiveBall = new SequentialCommandGroup(
                                getRangedAutoShoot().withTimeout(0.75),
                                ramsete.createRamseteCommand(PathPlannerPath.SHOOT_MIDBALL),
                                new InstantCommand(intake::intake, intake),
                                new ParallelCommandGroup(
                                                ramsete.createRamseteCommand(PathPlannerPath.MIDBALL_SHOOT),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(3),
                                                                new InstantCommand(intake::stop, intake),
                                                                getRangedAutoShoot().withTimeout(1.5))),
                                // ramsete.createRamseteCommand(PathPlannerPath.MIDBALL_WALLBALL),
                                // ramsete.createRamseteCommand(PathPlannerPath.WALLBALL_SHOOT)
                                // .andThen(new InstantCommand(intake::stop, intake)),
                                ramsete.createRamseteCommand(PathPlannerPath.SHOOT_WALL),
                                new InstantCommand(intake::intake, intake),
                                ramsete.createRamseteCommand(PathPlannerPath.WALL_TERMINAL),
                                // new ParallelCommandGroup(
                                // ramsete.createRamseteCommand(PathPlannerPath.SHOOT_TERMINAL),
                                // new SequentialCommandGroup(
                                // new WaitCommand(1),
                                // new InstantCommand(intake::intake, intake))),
                                // ramsete.createRamseteCommand(PathPlannerPath.TERMINAL_WALL),
                                // ramsete.createRamseteCommand(PathPlannerPath.WALL_SHOOT),
                                new ParallelCommandGroup(
                                                ramsete.createRamseteCommand(PathPlannerPath.TERMINAL_SHOOT),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new InstantCommand(intake::stop, intake),
                                                                new WaitCommand(2.2),
                                                                getAutoShoot())));

                twoBallFar = new SequentialCommandGroup(
                                new InstantCommand(intake::intake),
                                ramsete.createRamseteCommand(PathPlannerPath.TWO_BALL_FAR),
                                getAutoShoot().withTimeout(2),
                                new ParallelCommandGroup(
                                                new InstantCommand(intake::toggleIntake, intake),
                                                ramsete.createRamseteCommand(PathPlannerPath.FAR_DEFENSE)),
                                new InstantCommand(intake::outtake));

                oneBallFarMid = new SequentialCommandGroup(
                                getAutoShoot().withTimeout(5),
                                ramsete.createRamseteCommand(PathPlannerPath.ONE_BALL_FAR_MID));

                oneBallFarFar = new SequentialCommandGroup(
                                getAutoShoot().withTimeout(5),
                                ramsete.createRamseteCommand(PathPlannerPath.ONE_BALL_FAR_FAR));

                pathArrayMap.put(fiveBall, fiveBallPaths);
                pathArrayMap.put(twoBallFar, twoBallFarPaths);
                pathArrayMap.put(oneBallFarFar, oneBallFarFarPaths);
                pathArrayMap.put(oneBallFarMid, oneBallFarMidPaths);
        }

        public void setInitialDrivePose(SequentialCommandGroup auto) {
                if (pathArrayMap.containsKey(auto)) {
                        drivetrain.setOdometry(pathArrayMap.get(auto)[0].getTrajectory());
                }
        }
}