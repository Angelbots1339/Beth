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
    public final CANSparkMax moveMotor;
    public final CANSparkMax rotateMotor;
    public final WPI_CANCoder rotationEncoder;
    public final SwerveConfiguration swerveConfiguration;
    public final PIDController pidController;
    public final SwerveConfiguration config;

    public SwerveModule(
            SwerveConfiguration swerveConfiguration
    ) {
        this.config = swerveConfiguration;
        this.moveMotor = new CANSparkMax(swerveConfiguration.moveCANSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rotateMotor = new CANSparkMax(swerveConfiguration.rotateCANSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rotationEncoder = new WPI_CANCoder(swerveConfiguration.canCoderId);
        this.swerveConfiguration = swerveConfiguration;
        this.pidController = new PIDController(0.01, 0, 0.0001);
    }

    public Translation2d getPosition() {
        return this.swerveConfiguration.position;
    }

    public void init() {
        setupCANSparkMax(this.moveMotor);
        moveMotor.setInverted(false);

        setupCANSparkMax(this.rotateMotor);
        rotateMotor.setInverted(true);

        CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
        _canCoderConfiguration.unitString = "deg";
        _canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        rotationEncoder.configAllSettings(_canCoderConfiguration);
        rotationEncoder.configMagnetOffset(swerveConfiguration.magneticOffset);

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

    public void apply(SwerveModuleState state) {
        double encoderAbsolutePosition = rotationEncoder.getAbsolutePosition();
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
            rotateMotor.set(pid);
        }

        // TODO convert meters per second into motor speed and adjust clamp to be based on that
        moveMotor.set(mps);
    }
}
