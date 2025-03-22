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

    private int startDelay = 10;

    private boolean isL1;

    private double speed = -1;

    public ManipulateCoral(boolean isL1) {

        this.isL1 = isL1;

        intakeSubsystem = IntakeSubsystem.getInstance();

        if (isL1) {
            speed = 0.5;
        }

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (startDelay <= 0 && !isL1) {
            startDelay--;
            return;
        }

        if (PivotSubsystem.nearSetpoint() && ExtensionSubsystem.nearSetpoint() && ManipulatorSubsystem.nearSetpoint()) {
            if (!isL1) {
                timer++;
            }
            intakeSubsystem.setIntakeSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return timer >= 30 || (intakeSubsystem.hasPiece() && isL1);
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntakeSpeed(0);

        timer = 0;
        startDelay = 10;
    }
}