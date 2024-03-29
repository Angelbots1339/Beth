// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RobotInitCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.swervesetup.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PowerSubsystem;

import java.util.function.Supplier;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Pigeon2 pigeon2;
    private final PowerSubsystem powerSubsystem;
    private final SwerveSubsystem swerveSubsystem;

//    public final SwerveCommand swerveCommand;

    private final CommandXboxController m_driverController;
    private Rotation2d lastAngle = new Rotation2d();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        pigeon2 = new Pigeon2(1);
        pigeon2.configMountPose(Pigeon2.AxisDirection.PositiveY, Pigeon2.AxisDirection.PositiveZ);
        pigeon2.setYaw(0);

        m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

        powerSubsystem = new PowerSubsystem();
        swerveSubsystem = new SwerveSubsystem();

        Supplier<Double> joystickRobotSpin = () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.1);

        Supplier<Rotation2d> robotHeadingAngle = () -> {
            double yaw = pigeon2.getYaw();
            while (yaw >= 360.0) {
                yaw -= 360.0;
            }
            while (yaw <= 0) {
                yaw += 360.0;
            }
            return Rotation2d.fromDegrees(yaw);
        };
        Supplier<Translation2d> joystickRobotMovement = () -> new Translation2d(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.2),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.2)
        );
//        swerveCommand = new SwerveCommand(
//                driveSubsystem,
//                robotHeadingAngle,
//                joystickRobotSpin,
//                robotMovement
//        );

        // Configure the trigger bindings
        configureBindings();
//        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, swerveCommand);
        swerveSubsystem.setDefaultCommand(
                new SwerveCommand(
                        swerveSubsystem,
                        robotHeadingAngle,
                        joystickRobotSpin,
                        joystickRobotMovement
                )
        );
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_driverController.start().onTrue(Commands.runOnce(() -> {
            pigeon2.setYaw(0);
            lastAngle = Rotation2d.fromDegrees(0);
        }));

        m_driverController.povUp().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(0)));
        m_driverController.povUpLeft().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(45)));
        m_driverController.povLeft().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(90)));
        m_driverController.povDownLeft().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(135)));
        m_driverController.povDown().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(180)));
        m_driverController.povDownRight().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(-135)));
        m_driverController.povRight().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(-90)));
        m_driverController.povUpRight().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(-45)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
//    return Autos.exampleAuto(driveSubsystem);
        return null;
    }

    public RobotInitCommand getRobotInitCommand() {
        return new RobotInitCommand(
                swerveSubsystem,
                powerSubsystem
        );
    }
}
