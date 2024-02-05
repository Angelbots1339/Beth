package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {
    private final PowerDistribution powerDistributionHub;

    public PowerSubsystem() {
        powerDistributionHub = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    }

    public void robotInit() {
        powerDistributionHub.clearStickyFaults();  // TODO Fetch and log these?
    }
}
