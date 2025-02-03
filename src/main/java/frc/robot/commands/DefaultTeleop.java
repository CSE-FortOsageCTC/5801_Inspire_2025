package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public class DefaultTeleop extends Command{


    private CommandSwerveDrivetrain driveTrain;
    private int translationSup;
    private int strafeSup;
    private int rotationSup;
    private int throttle;
    private boolean robotCentricSup;
    private int back;
    private Joystick driver;
    private Joystick operator;

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0); 
    private SlewRateLimiter throttleLimiter = new SlewRateLimiter(1);
    private Pose2d alignPose;

    /* Setting up bindings for necessary control of the swerve drive platform */
    
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public DefaultTeleop(Joystick driver, Joystick operator, CommandSwerveDrivetrain driveTrain) {
        
        this.driver = driver;
        this.operator = operator;
        this.driveTrain = driveTrain;
        throttle = XboxController.Axis.kRightTrigger.value;
        translationSup = XboxController.Axis.kLeftY.value;
        strafeSup = XboxController.Axis.kLeftX.value;
        rotationSup = XboxController.Axis.kRightX.value;
        back = XboxController.Button.kBack.value;
        robotCentricSup = true;
        addRequirements(driveTrain);

    }

    @Override
    public void initialize() {
        
    } 
    

    @Override
    public void execute() { 
        Alliance alliance = DriverStation.getAlliance().get();
        double yAxis = alliance.equals(Alliance.Red) ? driver.getRawAxis(translationSup) : -driver.getRawAxis(translationSup);
        double xAxis = alliance.equals(Alliance.Red) ? driver.getRawAxis(strafeSup) : -driver.getRawAxis(strafeSup);
        double rotationAxis = driver.getRawAxis(rotationSup);

        double translationVal = MathUtil.applyDeadband(yAxis, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(xAxis, Constants.stickDeadband);
        double rotationVal;


        double throttleAxis = driver.getRawAxis(throttle);

        throttleAxis = (Math.abs(throttleAxis) < Constants.stickDeadband) ? .2 : throttleAxis;
        rotationAxis = (Math.abs(rotationAxis) < Constants.stickDeadband) ? 0 : rotationAxis;

        //if (AlignPosition.getPosition().equals(AlignPosition.Manual)) {
            rotationVal = rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));
            robotCentricSup = true;
        // } else if (AlignPosition.getPosition().equals(AlignPosition.SpeakerPos) || AlignPosition.getPosition().equals(AlignPosition.StagePos)){
        //     rotationVal = s_DefaultTeleop.s_Swerve.rotateToPos(); // s_AutoRotateUtil.calculateRotationSpeed();
        //     robotCentricSup = true;
        // } else if (AlignPosition.getPosition().equals(AlignPosition.AmpPos)) {
        //     rotationVal = s_DefaultTeleop.s_Swerve.rotateToAmp();
        //     robotCentricSup = true;
            
        // } else if (AlignPosition.getPosition().equals(AlignPosition.AutoPickup)) {
        //    rotationVal = s_DefaultTeleop.s_Swerve.rotateToNote();
        //    robotCentricSup = false;
        // } else {
        //     rotationVal = rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));
        //     robotCentricSup = true;
        // }
    
        double throttleCalc = throttleLimiter.calculate(throttleAxis);

        Translation2d translation = new Translation2d(translationVal, strafeVal).times(-maxSpeed * throttleCalc);
        
        //s_Swerve.drive(translation,  rotationVal * (driver.getRawButton(back)? Constants.Swerve.panicRotation:Constants.Swerve.maxAngularVelocity), robotCentricSup, true);
        driveTrain.setControl(
        drive.withVelocityX(translation.getX()) // Drive forward with negative Y (forward)
            .withVelocityY(translation.getY()) // Drive left with negative X (left)
            .withRotationalRate(rotationVal * (maxAngularRate)) // Drive counterclockwise with negative X (left)
    );

    }

    @Override
    public void end(boolean isFinished) {

    }
}