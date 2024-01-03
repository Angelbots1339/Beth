package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveConfiguration;
import frc.robot.subsystems.drive.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics m_kinematics;

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConfiguration.FRONT_LEFT);
        frontRight = new SwerveModule(SwerveConfiguration.FRONT_RIGHT);
        backLeft = new SwerveModule(SwerveConfiguration.BACK_LEFT);
        backRight = new SwerveModule(SwerveConfiguration.BACK_RIGHT);

        m_kinematics = new SwerveDriveKinematics(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        );
    }

    public SwerveModule[] getModules() {
        return new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void robotInit() {
        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();
    }

    public void apply(ChassisSpeeds speeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        frontLeft.apply(states[0]);
        frontRight.apply(states[1]);
        backLeft.apply(states[2]);
        backRight.apply(states[3]);
    }
}