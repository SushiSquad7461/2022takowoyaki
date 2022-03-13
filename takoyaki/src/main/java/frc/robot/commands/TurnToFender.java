// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.utils.TunableNumber;

/** An example command that uses an example subsystem. */
public class TurnToFender extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private final PhotonCamera camera;
    private final PIDController pid;
    private final Supplier<Double> linearVelocity;

    private TunableNumber kP = new TunableNumber("Turn to Fender P", Constants.kTurnToFender.kP);
    private TunableNumber kI = new TunableNumber("Turn to Fender I", Constants.kTurnToFender.kI);
    private TunableNumber kD = new TunableNumber("Turn to Fender D", Constants.kTurnToFender.kD);

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TurnToFender(Drivetrain drivetrain, PhotonCamera camera, Supplier<Double> linearVelocity) {
        pid = new PIDController(kP.get(), kI.get(), kD.get());
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.linearVelocity = linearVelocity;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            SmartDashboard.putBoolean("has targets", true);
            double yawSum = 0;
            for (PhotonTrackedTarget target : result.getTargets()) {
                yawSum += target.getPitch();
            }
            double yaw = yawSum / result.getTargets().size();
            double turnSpeed = pid.calculate(yaw, 0);
            SmartDashboard.putNumber("average yaw fan", yaw);
            drivetrain.curveDrive(linearVelocity.get(), turnSpeed, false);
        } else {
            SmartDashboard.putBoolean("has targets", false);
        }
        SmartDashboard.putBoolean("turning to fender", true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("turning to fender", false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
