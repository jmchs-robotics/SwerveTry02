package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class SetAngleCommand extends Command {

    private final SwerveDriveSubsystem drivetrain;
    private SwerveDriveModule[] mSwerveModules;
    private final double angleWant;

    public SetAngleCommand(SwerveDriveSubsystem drivetrain, double a) {
        this.drivetrain = drivetrain;
        this.angleWant = a;

        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        
        for (SwerveDriveModule module : drivetrain.getSwerveModules()) {
            module.setTargetAngle(angleWant);
        }
        // SmartDashboard.putNumber("YahoooooooooYEAH!!!!!!!@@#$@$@$$", 100);
    }

    @Override
    protected boolean isFinished() {

        return true;
    }

    @Override
    protected void end() {

    }

    @Override
    protected void interrupted() {
        end();
    }
}
