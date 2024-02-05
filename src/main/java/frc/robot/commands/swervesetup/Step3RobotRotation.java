package frc.robot.commands.swervesetup;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

/**
 * Step 3: Robot Rotation
 * <p>
 * In this step, we start to make sure that the robot rotates the correct way and the controller is set up correctly.
 * This command expects a "Supplier<Double>" from the joystick that you're using to control the robot's rotation. The
 * joystick is usually fully left == -1 and fully right == 1.
 * <p>
 * The ChassisSpeeds object expects rotation to be provided in "omega radians per second". Let's break that down. A
 * circle can be broken up into degrees, which have the values 0 to 360 for a circle. They can also be broken up into
 * radians which is between 0 and 2π. π == pi if you've not seen the symbol in text or if it renders differently in
 * your font. You can think about the conversion as every 180 degrees is one π radians.
 * <p>
 * In other words, passing a value of π into this function would cause the robot to rotate counterclockwise with enough
 * speed (assuming the motors can keep up) to spin halfway around within one second. This might be a little fast,
 * especially for testing. This class takes the value returned by the joystickLeftRightSupplier and multiplies it by
 * SPIN_ROTATIONS_PER_SECOND which has a value of 0.2. That ends up being a whopping 11.5 degrees per second, plenty
 * safe enough to test that rotation is working even if you decide to put your robot on the ground. Just for
 * completeness, -π would rotate clockwise.
 * <p>
 * Because moving the joystick left produces a negative value, but rotating left (counterclockwise) needs a positive
 * value, you generally want to negate whatever you get from your joystick when returning from your supplier
 * lambda/method.
 * <p>
 * !! Make sure EVERYTHING works perfectly in the steps before this. Otherwise, you'll be chasing your tail !!
 * <p>
 * Your end goal is to make the robot rotate to the left (counterclockwise) when the joystick goes left and to the right
 * (clockwise) when the joystick moves to the right. You'll need the joystickLeftRightSupplier for the DriveCommand.
 * <p>
 * One final thing, look at the concept of deadband: {@link edu.wpi.first.math.MathUtil#applyDeadband(double, double)}
 * A deadband is useful with joysticks because even without touching the joystick, you may get noise in the signal that
 * produces a non-zero value. If you let this go directly to your robot, your robot will jitter around. Applying a
 * small deadband value, such as 0.1, to your joystick means that only values between -1 and -0.1 or between 0.1 and 1
 * will be considered. The tradeoff to this is if you want to move your robot a teensy weensy bit, it won't respond
 * until the value is large enough to be outside the deadband. Use the dashboard value to test and determine a suitable
 * value.
 */
public class Step3RobotRotation extends Command {
    private static final double SPIN_ROTATIONS_PER_SECOND = 0.2;

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> joystickLeftRightSupplier;

    public Step3RobotRotation(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> joystickLeftRightSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.joystickLeftRightSupplier = joystickLeftRightSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double leftRightValue = joystickLeftRightSupplier.get();

        SmartDashboard.putNumber("joystick LR", leftRightValue);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                0,
                0,
                leftRightValue * SPIN_ROTATIONS_PER_SECOND
        );
        swerveSubsystem.apply(chassisSpeeds);
    }
}
