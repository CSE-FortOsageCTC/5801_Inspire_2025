package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class GoToArmPosition extends Command{
    
    private ArmPosition position;

    private PivotSubsystem pivotSubsystem;

    private ExtensionSubsystem extensionSubsystem;

    private ManipulatorSubsystem manipulatorSubsystem;

    public GoToArmPosition(ArmPosition position) {

        this.position = position;

        pivotSubsystem = PivotSubsystem.getInstance();
        extensionSubsystem = ExtensionSubsystem.getInstance();
        manipulatorSubsystem = ManipulatorSubsystem.getInstance();

        addRequirements(pivotSubsystem, extensionSubsystem, manipulatorSubsystem);
    }

    @Override
    public void execute() {
        pivotSubsystem.setPosition();
        extensionSubsystem.setPosition();
        manipulatorSubsystem.setPosition();
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atPosition() && extensionSubsystem.atPosition() && manipulatorSubsystem.atPosition();
    }

    @Override
    public void end(boolean isFinished) {
        pivotSubsystem.setSpeed(0);
        extensionSubsystem.setSetpoint(0);
        manipulatorSubsystem.setWristSpeed(0);
    }

}
