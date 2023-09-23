package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    private final Pigeon2 pigeon2;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics m_kinematics;

    public DriveSubsystem() {
        pigeon2 = new Pigeon2(1);

        frontLeft = new SwerveModule(SwerveModule.SwervePosition.FRONT_LEFT);
        frontRight = new SwerveModule(SwerveModule.SwervePosition.FRONT_RIGHT);
        backLeft = new SwerveModule(SwerveModule.SwervePosition.BACK_LEFT);
        backRight = new SwerveModule(SwerveModule.SwervePosition.BACK_RIGHT);

        m_kinematics = new SwerveDriveKinematics(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        );
    }

    public void robotInit() {
        pigeon2.configMountPose(Pigeon2.AxisDirection.PositiveY, Pigeon2.AxisDirection.PositiveZ);
        pigeon2.setYaw(0);

        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();
    }

    public void periodic() {
//        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
        double direction = pigeon2.getYaw();
        double target = 0;
        double rotation;
        if (Math.abs(direction - target) < 1) {
            rotation = 0;
        } else if (direction > target) {
            rotation = target - direction;
        } else if (direction < target) {
            rotation = target - direction;
        } else {
            rotation = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                rotation,
                Rotation2d.fromDegrees(direction)
        );
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }
}