package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DefaultTeleop extends Command {

    private Swerve s_Swerve;
    private AutoRotateUtil s_AutoRotateUtil;

    private int translationSup;
    private int strafeSup;
    private int rotationSup;
    private int throttle;
    private boolean robotCentricSup;
    private int back;
    private Joystick driver;
    private Joystick operator;

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);
    private SlewRateLimiter throttleLimiter = new SlewRateLimiter(2);
    private Pose2d alignPose;
    private boolean isAprilAligning = false;

    public DefaultTeleop(Joystick driver, Joystick operator) {
        s_Swerve = Swerve.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0);
        this.driver = driver;
        this.operator = operator;
        throttle = XboxController.Axis.kRightTrigger.value;
        translationSup = XboxController.Axis.kLeftY.value;
        strafeSup = XboxController.Axis.kLeftX.value;
        rotationSup = XboxController.Axis.kRightX.value;
        back = XboxController.Button.kBack.value;
        robotCentricSup = true;
        addRequirements(s_Swerve);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance().get();
        // double yAxis = alliance.equals(Alliance.Red) ? driver.getRawAxis(translationSup)
                //  : -driver.getRawAxis(translationSup);
        double yAxis = driver.getRawAxis(translationSup);
        // double xAxis = alliance.equals(Alliance.Red) ? driver.getRawAxis(strafeSup) : -driver.getRawAxis(strafeSup);
        double xAxis = driver.getRawAxis(strafeSup);

        double rotationAxis = driver.getRawAxis(rotationSup);
        // SmartDashboard.putString("Teleop Alignment", alignPose == null ? "" :
        // alignPose.toString());

        double translationVal = MathUtil.applyDeadband(yAxis, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(xAxis, Constants.stickDeadband);
        double rotationVal;

        double throttleAxis = driver.getRawAxis(throttle);

        throttleAxis = (Math.abs(throttleAxis) < Constants.stickDeadband) ? .2 : throttleAxis;
        rotationAxis = (Math.abs(rotationAxis) < Constants.stickDeadband) ? 0 : rotationAxis;

        rotationVal = rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));
        robotCentricSup = true;

        double throttleCalc = throttleLimiter.calculate(throttleAxis);

        Translation2d translation = new Translation2d(translationVal, strafeVal)
                .times(-Constants.Swerve.maxSpeed * (throttleCalc * 1));

        s_Swerve.teleopDrive(translation, rotationVal * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);

    }

    @Override
    public void end(boolean isFinished) {
        s_AutoRotateUtil.end();
    }
}