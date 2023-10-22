package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics m_kinematics;

    public DriveSubsystem() {
        frontLeft = new SwerveModule(SwerveModule.SwerveConfiguration.FRONT_LEFT);
        frontRight = new SwerveModule(SwerveModule.SwerveConfiguration.FRONT_RIGHT);
        backLeft = new SwerveModule(SwerveModule.SwerveConfiguration.BACK_LEFT);
        backRight = new SwerveModule(SwerveModule.SwerveConfiguration.BACK_RIGHT);

        m_kinematics = new SwerveDriveKinematics(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        );
    }

    public void robotInit() {
        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();
    }

    public void runSwerve(ChassisSpeeds speeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }
}