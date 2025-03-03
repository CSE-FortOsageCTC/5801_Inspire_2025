package frc.robot.autoCommands;

import java.security.cert.Extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ManipulateCoral extends Command {

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
        if (PivotSubsystem.nearSetpoint() && ExtensionSubsystem.nearSetpoint() && ManipulatorSubsystem.nearSetpoint()) {
            if (!intaking) {
                timer++;
            }
            intakeSubsystem.setIntakeSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return timer >= 40 || (intakeSubsystem.hasPiece() && intaking);
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntakeSpeed(0);
    }

}
