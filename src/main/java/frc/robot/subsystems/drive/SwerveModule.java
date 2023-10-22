package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax move;
    private final CANSparkMax rotate;
    private final WPI_CANCoder encoder;
    private final SwerveConfiguration swerveConfiguration;
    private final PIDController pidController;

    public enum SwerveConfiguration {
        FRONT_LEFT(
                "Front Left",
                new Translation2d(-0.25, 0.25),
                13,
                18,
                8,
                -95
        ),
        FRONT_RIGHT(
                "Front Right",
                new Translation2d(0.25, 0.25),
                12,
                15,
                9,
                -177
        ),
        BACK_LEFT(
                "Back Left",
                new Translation2d(-0.25, -0.25),
                20,
                17,
                6,
                56
        ),
        BACK_RIGHT(
                "Back Right",
                new Translation2d(0.25, -0.25),
                30,
                14,
                7,
                -71
        );

        // If gears face inward on wheels at offset, then move motion should be inverted for left from right

        public final String name;
        public final Translation2d position;
        public final int moveCANSparkMaxId;
        public final int rotateCANSparkMaxId;
        public final int canCoderId;
        public final double magneticOffset;

        SwerveConfiguration(
                String name,
                Translation2d position,
                int moveCANSparkMaxId,
                int rotateCANSparkMaxId,
                int CANCoderId,
                double magneticOffset
        ) {
            this.name = name;
            this.position = position;
            this.moveCANSparkMaxId = moveCANSparkMaxId;
            this.rotateCANSparkMaxId = rotateCANSparkMaxId;
            this.canCoderId = CANCoderId;
            this.magneticOffset = magneticOffset;
        }
    }

    public SwerveModule(
            SwerveConfiguration swerveConfiguration
    ) {
        this.move = new CANSparkMax(swerveConfiguration.moveCANSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rotate = new CANSparkMax(swerveConfiguration.rotateCANSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.encoder = new WPI_CANCoder(swerveConfiguration.canCoderId);
        this.swerveConfiguration = swerveConfiguration;
        this.pidController = new PIDController(0.01, 0.000001, 0);
    }

    public Translation2d getPosition() {
        return this.swerveConfiguration.position;
    }

    public void init() {
        setupCANSparkMax(this.move);
//        move.setInverted(swerveConfiguration.moveMotorInverted);

        setupCANSparkMax(this.rotate);
        rotate.setInverted(true);

        CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
        _canCoderConfiguration.unitString = "deg";
        _canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
//        _canCoderConfiguration.sensorDirection = true;

        encoder.configAllSettings(_canCoderConfiguration);
        encoder.configMagnetOffset(swerveConfiguration.magneticOffset);

        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(5, 10);
    }

    private void setupCANSparkMax(CANSparkMax sparkMax) {
        sparkMax.clearFaults();
        sparkMax.restoreFactoryDefaults();
        sparkMax.setSmartCurrentLimit(30);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        sparkMax.enableVoltageCompensation(12);
    }

    public void setState(SwerveModuleState state) {
        double encoderAbsolutePosition = encoder.getAbsolutePosition();
        Rotation2d position = Rotation2d.fromDegrees(encoderAbsolutePosition);
        state = SwerveModuleState.optimize(state, position);

        double mps = MathUtil.clamp(state.speedMetersPerSecond, -1, 1);
        SmartDashboard.putNumber(String.format("%s mps", swerveConfiguration.name), mps);
        SmartDashboard.putNumber(String.format("%s deg", swerveConfiguration.name), state.angle.getDegrees());
        SmartDashboard.putNumber(String.format("%s encoder", swerveConfiguration.name), encoderAbsolutePosition);

        double pid = pidController.calculate(encoderAbsolutePosition, state.angle.getDegrees());
        SmartDashboard.putNumber(String.format("%s pid", swerveConfiguration.name), pid);
        SmartDashboard.putNumber(String.format("%s pid error", swerveConfiguration.name), pidController.getPositionError());
        if (!pidController.atSetpoint()) {
            rotate.set(pid);
        }

        // TODO convert meters per second into motor speed and adjust clamp to be based on that
        move.set(mps);
    }
}
