package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    public double speed;

    public IntakeCommand(double speed) {
        this.speed = speed;

        intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntakeSpeed(0);
    }
}
