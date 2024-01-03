package frc.robot.commands.swervesetup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;

/**
 * Swerve Setup step 2: Rotation Tuning
 * <p>
 * Rotation tuning values should be the same between all of your swerve modules. If you get wildly different responses,
 * check the hardware and make sure everything is assembled properly. In general, this will be tuning a PID controller,
 * almost always the "P" and "D" portions. The "I" portion isn't usually needed.
 * <p>
 * One thing to note is that the previous step controlled the move motor directly. This and further steps will use the
 * {@link SwerveModule#apply(SwerveModuleState)} method to control the swerve module. You can ignore the fact that the
 * speed parameter is "meters per second". Just set your move motor to that value directly, we'll use 0.1 in these
 * steps until specified otherwise.
 * <p>
 * For now, our goal is to make sure we can specify a give rotation angle for each module and have the modules follow it
 * as precisely as we can. This command starts with having the motors point forward in the positive X direction and then
 * rotates them by 90 degrees every 10 seconds. Your end result should be the motors rotating quickly to the correct
 * rotation without much oscillation.
 * <p>
 * !! Make sure your PID controller continuous input enabled !!
 * !! Make sure you use {@link edu.wpi.first.math.MathUtil#clamp(double, double, double)} to clamp your output between
 * -1 and 1, so you don't overpower or fry your motor !!
 * <p>
 * Binary search: If you understand binary search, you can skip this section. A binary search is a fast way to narrow
 * in on specific values when you can determine if a given test number is too high or low. For example, say you had the
 * range 1 to 100 inclusive. If we called 1 our "minimum" and 100 our "maximum", we can test the number in the middle:
 * (minimum + maximum) / 2
 * In our case, that's going to be 50 since these are integers. If we can tell that 50 is too low, that wipes out all
 * the numbers 1-49. Our new minimum is 50, and we can repeat this process. If the next test, 75, is too high then we
 * can lower the maximum to 75 and repeat. Eventually we'll be left with only one number. Doing the same thing with
 * floating point values works until you have enough precision, which in our application can be as little as one
 * significant digit.
 * <p>
 * Start with all PID values at 0. To tune "P", start with a very, very low decimal value. Something like 0.0000001
 * should do the trick. Multiply it by 10 until the wheel actually starts rotating. If you go too far, you'll induce a
 * lot of nasty oscillations (though you can actually induce those oscillations even at lower "P" values if the rotate
 * motor needs to be inverted). For your binary search, too high means "bad oscillations" and too low means "doesn't get
 * there as fast as it can".
 * <p>
 * Next up, do the same with the "D" parameter. This will dampen the oscillations, and we essentially want to see them
 * go away. You want to make sure that they do get close to the target still. A "too high" value here means we dampen
 * too much before we get to our set point, but too little, and we'll still see the oscillations.
 * <p>
 * Finally, the "I" parameter corrects when we're not quite at our target. In general, it's easier to modify the P and D
 * parameters to hit the target or close enough to it, than to modify the "I" parameter. The main thing you want to
 * avoid by increasing this value too high is <a href="https://en.wikipedia.org/wiki/Integral_windup">Integral Windup</a>
 * Too low of an "I" value just means you're not correcting fast enough to get to your target. I don't generally
 * recommend messing with this value.
 * <p>
 * One final note on all of this is that you're probably still testing the robot with the wheels not on the ground. This
 * is totally fine for our purposes and is still recommended for now. However, when the robot is put on the ground and
 * all the weight of the robot is put on those wheels, it may change your response values,  and you may have to run
 * through these tuning steps again. Luckily most of the tuning can be done purely based on the dashboard outputs.
 */
public class Step2RotationTuning extends CommandBase {
    static final int PAUSE_COUNT = 10_000 / 20;  // 10 seconds split into 20 ms chunks
    private final SwerveSubsystem swerveSubsystem;
    private int counter = 0;
    private int desiredAngle = 0;

    public Step2RotationTuning(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Desired Angle", desiredAngle);

        counter++;
        if (counter >= PAUSE_COUNT) { // 10 seconds
            counter = 0;
            desiredAngle = (desiredAngle + 90) % 360;
        }

        for (SwerveModule module : swerveSubsystem.getModules()) {
            SmartDashboard.putNumber(module.config.name + " rotation", module.rotationEncoder.getAbsolutePosition() * 360);

            // Make each module move forward slowly, so you can determine the direction
            module.apply(new SwerveModuleState(0.1, Rotation2d.fromDegrees(desiredAngle)));
        }
    }
}
