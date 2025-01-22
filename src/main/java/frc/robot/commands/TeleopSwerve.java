package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;


public class TeleopSwerve extends Command {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private int throttle;

    //private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
	//private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(1.8); 
    private SlewRateLimiter throttleLimiter = new SlewRateLimiter(2); 

    @Override
    public void initialize() {

    }
    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, int throttle) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.throttle = throttle;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        double throttleAxis = controller.getRawAxis(throttle);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        throttleAxis = (Math.abs(throttleAxis) < Constants.stickDeadband) ? .2 : throttleAxis;

        translation = new Translation2d(yAxis, xAxis).times(throttleLimiter.calculate(throttleAxis) * (5));
        rotation = rotationLimiter.calculate(rAxis) * (throttleLimiter.calculate(throttleAxis) * (Constants.Swerve.maxAngularVelocity));
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
