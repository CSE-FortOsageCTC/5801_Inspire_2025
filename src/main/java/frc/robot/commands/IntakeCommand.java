package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    public boolean intaking;
    public double speed;

    public IntakeCommand(boolean intaking) {
        this.intaking = intaking;

        intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (intaking) {
            speed = -1;
        } else if (!intaking && ArmPosition.getPosition().equals(ArmPosition.L1)) {
            speed = 0.25;
        } else if (!intaking) {
            speed = 1;
        }

        intakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntakeSpeed(0);
    }
}
