
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.List;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.Constants.kTurnToTarget;

public class TurnToTarget extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private PhotonCamera camera;
    private Drivetrain drivetrain;
    private PIDController PID;

    /**
     * Creates a new TurnToTarget.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TurnToTarget(PhotonCamera camera, Drivetrain drivetrain) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        PID = new PIDController(kTurnToTarget.kP, kTurnToTarget.kI, kTurnToTarget.kD);
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
        List<PhotonTrackedTarget> targets = result.getTargets();
        double yaw = getYawFromAverage(targets); // change this to getYawFromHighest maybe ?!?!
        double output = PID.calculate(yaw, 0);
        drivetrain.curveDrive(0, output, true);
    }

    public double getYawFromAverage(List<PhotonTrackedTarget> targets) {
        double result = 0;
        for (PhotonTrackedTarget target : targets) {
            result += target.getYaw();
        }
        return result / targets.size();
    }

    public double getYawFromHighest(List<PhotonTrackedTarget> targets) {
        double highestPitch = 0;
        double highestYaw = Double.MIN_VALUE;
        for (PhotonTrackedTarget target : targets) {
            if (highestYaw < target.getYaw()) {
                highestPitch = target.getPitch();
            }
        }
        return highestPitch;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
