package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerSubsystem;

public class RobotInitCommand extends CommandBase {
    private boolean isFinished = false;
    private final DriveSubsystem driveSubsystem;
    private final PowerSubsystem powerSubsystem;

    public RobotInitCommand(
            DriveSubsystem driveSubsystem,
            PowerSubsystem powerSubsystem
    ) {
        this.driveSubsystem = driveSubsystem;
        this.powerSubsystem = powerSubsystem;
        addRequirements(this.powerSubsystem);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        driveSubsystem.robotInit();
        powerSubsystem.robotInit();
        isFinished = true;
    }
}
