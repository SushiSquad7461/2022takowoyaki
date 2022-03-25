package frc.robot;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

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

public class AutoCommandSelector {
        private final Drivetrain drivetrain;
        private final Ramsete ramsete;
        private final Intake intake;
        private final Shooter shooter;
        private final Hopper hopper;

        // sequential command groups
        public final SequentialCommandGroup fiveBall;

        public final PathPlannerPath[] fiveBallPaths = {
            PathPlannerPath.SHOOT_MIDBALL,
            PathPlannerPath.MIDBALL_WALLBALL,
            PathPlannerPath.WALLBALL_SHOOT,
            PathPlannerPath.SHOOT_TERMINAL,
            PathPlannerPath.TERMINAL_SHOOT
        };

        public final Map<SequentialCommandGroup, PathPlannerPath[]> pathArrayMap;

        private AutoShoot getAutoShoot() {
                return new AutoShoot(shooter, hopper, intake);
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
                                getAutoShoot().withTimeout(2),
                                ramsete.createRamseteCommand(PathPlannerPath.SHOOT_TERMINAL),
                                new InstantCommand(intake::toggleIntake, intake),
                                ramsete.createRamseteCommand(PathPlannerPath.MIDBALL_WALLBALL),
                                ramsete.createRamseteCommand(PathPlannerPath.WALLBALL_SHOOT)
                                                .andThen(new InstantCommand(intake::toggleIntake, intake)),
                                getAutoShoot().withTimeout(2),
                                new ParallelCommandGroup(
                                    ramsete.createRamseteCommand(PathPlannerPath.SHOOT_TERMINAL),
                                    new InstantCommand(intake::toggleIntake, intake)),
                                new WaitCommand(0.5),
                                new InstantCommand(intake::toggleIntake, intake),
                                ramsete.createRamseteCommand(PathPlannerPath.TERMINAL_SHOOT),
                                getAutoShoot());

                pathArrayMap.put(fiveBall, fiveBallPaths);
        }

        public void setInitialDrivePose(SequentialCommandGroup auto) {
                if (pathArrayMap.containsKey(auto)) {
                        drivetrain.setOdometry(pathArrayMap.get(auto)[0].getTrajectory());
                }
        }
}