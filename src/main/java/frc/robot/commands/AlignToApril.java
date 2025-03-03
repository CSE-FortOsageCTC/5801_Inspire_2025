package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Swerve;

public class AlignToApril extends Command {

    private boolean isScoring;
    private AlignPosition alignPos;
    private Swerve s_Swerve;

    private Translation2d translation;
    private double rotation;

    public AlignToApril(AlignPosition alignPos, boolean isScoring) {

        this.isScoring = isScoring;
        this.alignPos = alignPos;
        s_Swerve = Swerve.getInstance();

        addRequirements(s_Swerve);

    }

    @Override
    public void initialize() {
        s_Swerve.alignAprilTag(alignPos, isScoring);
    }

    @Override
    public void execute() {
        Rotation2d rotationTag = AlignPosition.getAlignOffset().getRotation();
        translation = s_Swerve.translateToApril().times(Constants.Swerve.maxSpeed);
        if (!isScoring) {
            rotationTag = rotationTag.rotateBy(Rotation2d.fromDegrees(180));
        }
        rotation = s_Swerve.rotateToApril(rotationTag.getDegrees()) * Constants.Swerve.maxAngularVelocity;
        s_Swerve.teleopDrive(translation, rotation, true, false);

    }

    @Override
    public boolean isFinished() {
        return s_Swerve.alignAprilFinished();
    }

    @Override
    public void end(boolean isFinished) {
        AlignPosition.setIsScoring(true);
        s_Swerve.teleopDrive(new Translation2d(0, 0), 0, true, false);
        s_Swerve.resetAlignApril();
    }

}
