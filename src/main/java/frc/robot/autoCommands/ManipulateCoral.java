package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ManipulateCoral extends Command{
    

    private IntakeSubsystem intakeSubsystem;

    private int timer = 0;

    private boolean intaking;

    private double speed = 1;

    public ManipulateCoral(boolean intaking) {

        this.intaking = intaking;

        intakeSubsystem = IntakeSubsystem.getInstance();

        if (intaking) {
            speed *= -1;
        }

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!intaking){
            timer++;
        }
        intakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return timer >= 50 || (intakeSubsystem.hasPiece() && intaking);
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntakeSpeed(0);
    }


}
