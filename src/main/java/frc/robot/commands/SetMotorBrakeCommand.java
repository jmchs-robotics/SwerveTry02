package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.subsystems.HolonomicDrivetrain;
import frc.robot.subsystems.SwerveDriveSubsystem;

public final class SetMotorBrakeCommand extends Command {
    /**
     * Creating a command for the setBrake Method in SwerveDriveModules
     * Sets it to the given boolean
     * 
     * Needs to pass through a robot
     */

    //private final HolonomicDrivetrain drivetrain;
   // private final boolean isFieldOriented;
    boolean youWantBrake;
    Robot robot;

    //@Deprecated
    public SetMotorBrakeCommand(Robot r, boolean b) {
         this.youWantBrake = b;
         this.robot = r;
    }
        
     @Override
    protected void initialize() {
        robot.getDrivetrain().setBrake(youWantBrake);
    }

   /* public SetMotorBrakeCommand(HolonomicDrivetrain drivetrain, boolean isFieldOriented) {
        this.drivetrain = drivetrain;
        this.isFieldOriented = isFieldOriented;
    } */

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
