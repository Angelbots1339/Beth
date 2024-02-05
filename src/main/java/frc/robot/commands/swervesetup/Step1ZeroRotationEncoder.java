package frc.robot.commands.swervesetup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;

/**
 * Swerve Setup step 1: Zeroing the rotation encoder
 * <p>
 * This is part of the swerve setup utilities. When the robot is facing forward on the field from your driver station,
 * it is facing in the positive X direction and the positive Y direction is to your left. In a unit circle, we have an
 * X and Y axis as well. If you think about it in terms of degrees, a vector at 0 degrees is pointing in the positive X
 * direction. At 90 degrees, it's pointing in the positive Y direction. At 180 degrees, it's pointing back at you in the
 * negative X direction. And at 270 degrees it's pointing in the negative Y direction or to your right.
 * <p>
 * A unit circle is "counterclockwise positive" because as you increase the angle, you're moving counterclockwise. If
 * you think about looking down on the robot from above, this same thing is true. This command will help you verify that
 * the rotation sensor is reading counterclockwise and will help you set the magnetic offset. Make sure you set the
 * magnetic offset to 0 for each swerve configuration first!
 * <p>
 * The robot will alternate between moving each wheel "forward" for 10 seconds and pausing for 10 seconds. This test
 * requires the robot's wheels to not be touching the ground.
 * <p>
 * !!DO NOT TOUCH THE ROBOT WHILE THE WHEELS ARE MOVING !!
 * <p>
 * When the wheels are not moving, gently rotate them one at a time to first verify they are reading correctly. I like
 * to work in degrees, but sometimes an encoder will be from 0 to 1 or from -180 to 180. Either way, if you're looking
 * down from above the robot, rotating the wheel counterclockwise should increase the rotation reading in the dashboard.
 * <p>
 * Once you've verified the correct rotation, rotate each wheel so that they would drive the robot straight forward.
 * The idea behind this is we want the robot to be driving in the positive X direction when our encoder reads 0, but
 * each magnet is placed slightly differently and the encoders can be mounted slightly differently. Once you've gotten
 * each wheel to where it drives "forward", record the rotation values. The negative of this is usually your magnetic
 * offset. You can verify you got the offset correct by updating your swerve configuration and seeing if pointing the
 * wheels forward shows 0 for the rotation in the dashboard.
 */
public class Step1ZeroRotationEncoder extends Command {
    static final int PAUSE_COUNT = 10_000 / 20;  // 10 seconds split into 20 ms chunks
    private final SwerveSubsystem swerveSubsystem;
    private int counter = 0;
    private boolean moving = false;

    public Step1ZeroRotationEncoder(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        counter++;
        if (counter >= PAUSE_COUNT) {
            counter = 0;

            moving = !moving;  // Every 10 seconds, switch from not moving to moving or vice-versa
        }

        for (SwerveModule module : swerveSubsystem.getModules()) {
            SmartDashboard.putNumber(module.config.name + " rotation", module.rotationEncoder.getAbsolutePosition());

            if (moving) {
                module.moveMotor.set(0.1);  // Move at a slow speed, so you can tell which direction it's moving in
            } else {
                module.moveMotor.set(0);  // Stop the motor. This allows adjusting the rotation manually to find the magnetic offset
            }
        }
    }
}
