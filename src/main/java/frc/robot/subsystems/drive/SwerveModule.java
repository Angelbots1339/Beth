package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax move;
    private final CANSparkMax rotate;
    private final WPI_CANCoder encoder;
    private final SwervePosition swervePosition;

    public enum SwervePosition {
        FRONT_LEFT(
                "Front Left",
                new Translation2d(-0.25, 0.25),
                13,
                18,
                8,
                169.0,
                true,
                false
        ),
        FRONT_RIGHT(
                "Front Right",
                new Translation2d(0.25, 0.25),
                12,
                15,
                9,
                -82.0,
                true,
                false
        ),
        BACK_LEFT(
                "Back Left",
                new Translation2d(-0.25, -0.25),
                20,
                17,
                6,
                -30.0,
                true,
                true
        ),
        BACK_RIGHT(
                "Back Right",
                new Translation2d(-0.25, -0.25),
                30,
                14,
                7,
                17.0,
                false,
                true
        );

        public final String name;
        public final Translation2d position;
        public final int moveCANSparkMaxId;
        public final int rotateCANSparkMaxId;
        public final int canCoderId;
        public final double magneticOffset;
        public final boolean sensorDirection;
        private final boolean moveMotorInverted;

        SwervePosition(
                String name,
                Translation2d position,
                int moveCANSparkMaxId,
                int rotateCANSparkMaxId,
                int CANCoderId,
                double magneticOffset,
                boolean sensorDirection,
                boolean moveMotorInverted
        ) {
            this.name = name;
            this.position = position;
            this.moveCANSparkMaxId = moveCANSparkMaxId;
            this.rotateCANSparkMaxId = rotateCANSparkMaxId;
            this.canCoderId = CANCoderId;
            this.magneticOffset = magneticOffset;
            this.sensorDirection = sensorDirection;
            this.moveMotorInverted = moveMotorInverted;
        }
    }

    public SwerveModule(
            SwervePosition swervePosition
    ) {
        this.move = new CANSparkMax(swervePosition.moveCANSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rotate = new CANSparkMax(swervePosition.rotateCANSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.encoder = new WPI_CANCoder(swervePosition.canCoderId);
        this.swervePosition = swervePosition;
    }

    public Translation2d getPosition() {
        return this.swervePosition.position;
    }

    public void init() {
        setupCANSparkMax(this.move);
        move.setInverted(swervePosition.moveMotorInverted);

        setupCANSparkMax(this.rotate);
        rotate.setInverted(swervePosition.sensorDirection);

        CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
        _canCoderConfiguration.unitString = "deg";
        _canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        _canCoderConfiguration.sensorDirection = swervePosition.sensorDirection;

        encoder.configAllSettings(_canCoderConfiguration);
        encoder.configMagnetOffset(swervePosition.magneticOffset);
    }

    private void setupCANSparkMax(CANSparkMax sparkMax) {
        sparkMax.clearFaults();
        sparkMax.setSmartCurrentLimit(20);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        sparkMax.enableVoltageCompensation(12);
    }

    public void setState(SwerveModuleState state) {
//        SmartDashboard.putNumber(String.format("%s mps", swervePosition.name), state.speedMetersPerSecond);
//        SmartDashboard.putNumber(String.format("%s deg", swervePosition.name), state.angle.getDegrees());
//        SmartDashboard.putNumber(String.format("%s encoder", swervePosition.name), encoder.getAbsolutePosition());

        double position = encoder.getAbsolutePosition();
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(position));

        double target = state.angle.getDegrees();
        double diff = position - target;

        double scaleFactor = Math.min(Math.abs(diff), 50);
        scaleFactor *= Math.signum(diff);
        scaleFactor /= 100;

        if (Math.abs(diff) > 1) {
            rotate.set(scaleFactor);
        } else {
            rotate.stopMotor();
        }
        // 188 rpm -> 1 m/2

        move.set(state.speedMetersPerSecond);  // TODO Take into account gearing
    }
}
