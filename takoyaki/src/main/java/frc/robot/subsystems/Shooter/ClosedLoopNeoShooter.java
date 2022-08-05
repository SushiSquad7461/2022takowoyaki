package frc.robot.subsystems.Shooter;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.SliderAdjustableNumber;

public class ClosedLoopNeoShooter extends Shooter {
    private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
    private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
    private final CANSparkMax back = new CANSparkMax(Constants.kShooter.BACK_MOTOR_ID, MotorType.kBrushless);
    private final WPI_TalonSRX kicker = new WPI_TalonSRX(Constants.kShooter.KICKER_MOTOR_ID);
    private final RelativeEncoder backEncoder;
    private final PIDController backController;

    private double frontSetpointRPMWithOffset;
    private double backSetpointRPMWithOffset;
    private double backGoal = 0.0;
    private double frontGoal = 0.0;
    private SliderAdjustableNumber ampOffsetSlider = new SliderAdjustableNumber("amp offset", 0, -1,
            1, 0.1);

    private SliderAdjustableNumber ratioOffsetSlider = new SliderAdjustableNumber("ratio offset", 0, -3,
            3, 0.5);

    // private TunableNumber frontP = new TunableNumber("frontP",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kP);
    // private TunableNumber frontI = new TunableNumber("frontI",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kI);
    // private TunableNumber frontD = new TunableNumber("frontD",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kD);
    // private TunableNumber frontF = new TunableNumber("frontF",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kF);

    // private TunableNumber backP = new TunableNumber("frontP",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kP);
    // private TunableNumber backI = new TunableNumber("frontI",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kI);
    // private TunableNumber backD = new TunableNumber("frontD",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kD);
    // private TunableNumber backF = new TunableNumber("frontF",
    // Constants.kShooter.kDoubleClosedLoop.kFront.kF);

    public ClosedLoopNeoShooter() {

        left.setNeutralMode(NeutralMode.Coast);
        right.setNeutralMode(NeutralMode.Coast);
        back.setIdleMode(IdleMode.kCoast);
        kicker.setNeutralMode(NeutralMode.Brake);

        left.setInverted(true);
        right.setInverted(false);
        back.setInverted(false);
        kicker.setInverted(!Constants.kShooter.kKicker.KICKER_INVERSION);

        left.configSupplyCurrentLimit(Constants.currentLimit(40));
        right.configSupplyCurrentLimit(Constants.currentLimit(40));
        back.setSmartCurrentLimit(40);
        kicker.configSupplyCurrentLimit(Constants.currentLimit(30));

        backController = new PIDController(Constants.kShooter.kClosedLoopNeo.kBack.kP,
                Constants.kShooter.kClosedLoopNeo.kBack.kI, Constants.kShooter.kClosedLoopNeo.kBack.kD);
        right.follow(left);

        this.zeroSetpoint();
        backEncoder = back.getEncoder();
    }

    public void runShooter() {
        frontGoal = Constants.kShooter.kOpenLoop.SPEED;
        backGoal = Constants.kShooter.kOpenLoop.BACK_SPEED;
    }

    public void stopShooter() {
        frontGoal = 0;
        backGoal = 0;
    }

    public void runKicker() {
        kicker.set(ControlMode.PercentOutput, Constants.kShooter.kKicker.MOTOR_SPEED);
    }

    public void stopKicker() {
        kicker.set(ControlMode.PercentOutput, 0);
    }

    public void reverseKicker() {
        kicker.set(ControlMode.PercentOutput, -Constants.kShooter.kKicker.MOTOR_SPEED);
    }

    public void zeroSetpoint() {
        this.frontSetpointRPMWithOffset = 0;
        this.backSetpointRPMWithOffset = 0;
    }

    @Override
    public void setState(ShooterState state) {
        this.state = state;
        if (Constants.OUTREACH_MODE) {
            this.state = ShooterState.OUTREACH;
        }
        SmartDashboard.putBoolean("tuning enabled", true);
        this.frontSetpointRPMWithOffset = ((Constants.kShooter.kDoubleClosedLoop.SETPOINT_RPM)
                * (this.getAmp() + ampOffsetSlider.get()))
                + Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET_RPM;

        this.backSetpointRPMWithOffset = (((Constants.kShooter.kDoubleClosedLoop.SETPOINT_RPM)
                * (this.getAmp() + ampOffsetSlider.get()))
                // * (this.getAmp() + ampOffsetSlider.get())
                * (this.getRatio() + ratioOffsetSlider.get()))
                + Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET_RPM;

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("front shooter rpm", left.getSelectedSensorVelocity() * 600.0 / 2048.0);
        SmartDashboard.putNumber("front shooter setpoint", frontSetpointRPMWithOffset);
        SmartDashboard.putNumber("back shooter rpm", backEncoder.getVelocity()
                * 600.0 / 2048.0);
        SmartDashboard.putNumber("back shooter setpoint", backSetpointRPMWithOffset);
        SmartDashboard.putBoolean("at speed", isAtSpeed());

        if (frontSetpointRPMWithOffset == 0) { // assume both setpoints are zero
            left.set(ControlMode.Velocity, 0);
            back.set(backController.calculate(backEncoder.getVelocity(), 0));
        } else {
            left.set(ControlMode.Velocity, Constants.convertRPMToTrans(frontSetpointRPMWithOffset));
            back.set(backController.calculate(backEncoder.getVelocity(),
                    Constants.convertRPMToTrans(backSetpointRPMWithOffset)));
        }
    }

    public boolean isAtSpeed() {
        if (frontSetpointRPMWithOffset == 0 && backSetpointRPMWithOffset == 0)
            return false;

        double frontDiff, backDiff;

        frontDiff = Math
                .abs(Constants.convertRPMToTrans(frontSetpointRPMWithOffset) - left.getSelectedSensorVelocity());
        backDiff = Math.abs(Constants.convertRPMToTrans(backSetpointRPMWithOffset))
                - backEncoder.getVelocity();
        return frontDiff <= Constants
                .convertRPMToTrans(Constants.kShooter.kDoubleClosedLoop.kFront.ERROR_TOLERANCE)
                && backDiff <= Constants.convertTransToRPM(Constants.kShooter.kDoubleClosedLoop.kBack.ERROR_TOLERANCE);

    }

    private double getFrontSetpointGoal() {
        return (Constants.kShooter.kDoubleClosedLoop.SETPOINT_RPM)
                * getAmp();
    }

    private double getBackSetpointGoal() {
        return getFrontSetpointGoal() * getAmp() * getRatio();
    }

    public double getKickerOutput() {
        return kicker.getMotorOutputVoltage();
    }

    @Override
    public void simulationPeriodic() {
    }
}