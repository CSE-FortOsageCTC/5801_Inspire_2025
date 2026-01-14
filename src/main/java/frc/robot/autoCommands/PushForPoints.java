package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PushForPoints extends Command {
    
    private Swerve swerve;

    private int counter = 0;

    private boolean isRed;

    private double speed = 0.2;

    public PushForPoints() {

        swerve = Swerve.getInstance();

        isRed = DriverStation.getAlliance().get().equals(Alliance.Red);

        if (isRed) {
            speed *= -1;
        }

    }

    @Override
    public void initialize() {
        swerve.drive(new Translation2d(speed, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
    }

    @Override
    public void execute() {

        if (counter >= 20) {
            swerve.drive(new Translation2d(-speed, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
        }

        counter++;

    }

    @Override
    public boolean isFinished() {
        return counter > 40;
    }

    @Override
    public void end(boolean isFinished) {
        swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
    }



}
