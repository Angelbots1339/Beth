package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PowerSubsystem;

public class RobotInitCommand extends CommandBase {
    private boolean isFinished = false;
    private final SwerveSubsystem swerveSubsystem;
    private final PowerSubsystem powerSubsystem;

    public RobotInitCommand(
            SwerveSubsystem swerveSubsystem,
            PowerSubsystem powerSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.powerSubsystem = powerSubsystem;
        addRequirements(this.powerSubsystem);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        swerveSubsystem.robotInit();
        powerSubsystem.robotInit();
        isFinished = true;
    }
}
