package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToApril extends Command{
    
    private boolean isLeft;
    private Swerve s_Swerve;

    private Translation2d translation;
    private double rotation;

    public AlignToApril(boolean isleft) {

        this.isLeft = isleft;
        s_Swerve = Swerve.getInstance();

        addRequirements(s_Swerve);

    }

    @Override
    public void initialize() {
        System.out.println("Initialized Align to April Command");
        s_Swerve.alignAprilTag(isLeft);
    }

    @Override
    public void execute() {
        
        translation = s_Swerve.translateToApril().times(Constants.Swerve.maxSpeed);
        rotation = s_Swerve.rotateToApril(AlignPosition.getAlignOffset().getRotation().getDegrees()) * Constants.Swerve.maxAngularVelocity;
        s_Swerve.teleopDrive(translation, rotation, true, false);

    }

    @Override
    public boolean isFinished() {
        return s_Swerve.alignAprilFinished();
    }

    @Override
    public void end(boolean isFinished) {
        System.out.println("Ended Align to April Command");
        s_Swerve.resetAlignApril();
    }



}
