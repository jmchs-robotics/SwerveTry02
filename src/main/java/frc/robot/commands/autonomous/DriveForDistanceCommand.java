package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class DriveForDistanceCommand extends Command {
    private static final double TARGET_DISTANCE_BUFFER = 4; // 2;
    private static final double DISTANCE_CHECK_TIME = 0.25;

    private final SwerveDriveSubsystem drivetrain;
    private final double angle;
    private final double distance;
    private final double distRight, distForward;
    private final PIDController angleErrorController;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    private double initialDrivetrainAngle = 0;
    private double rotationFactor = 0;

    private BufferedWriter[] encPosLoggers = new BufferedWriter[4];
    private BufferedWriter[] encVelLoggers = new BufferedWriter[4];
    private int iterCount;

    /**
     * Move the robot forward/backwards this many inches. 
     * Field oriented.
     * @param drivetrain 
     * @param distance inches; positive if forwards, negative is backwards.
     */
    public DriveForDistanceCommand(SwerveDriveSubsystem drivetrain, double distance) {
        this(drivetrain, 0, distance);
    }

    /**
     * Move the robot this many inches.  Robot does not turn, just strafes.
     * Field oriented.
     * @param drivetrain
     * @param distRight inches positive is to the right; negative is to the left.
     * @param distForward inches positive is forward; negaitve is backwards.
     */
    public DriveForDistanceCommand(SwerveDriveSubsystem drivetrain, double distRight, double distForward) {
        this.drivetrain = drivetrain;
        this.angle = Math.toDegrees(Math.atan2(distRight, distForward));

        this.distRight = distRight; // -distRight;  // 191206 seems like this should not get inverted
        this.distForward = distForward;

        this.distance = Math.sqrt(distRight * distRight + distForward * distForward);
        // 191206 this PID isn't working... probably needs more P.  Original is 0.02, 0, 0
        angleErrorController = new PIDController(0.02, 0, 0, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) { }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return drivetrain.getGyroAngle();
            }
        }, output -> {
            rotationFactor = output;
        });
        angleErrorController.setInputRange(0, 360);
        angleErrorController.setOutputRange(-0.5, 0.5);
        angleErrorController.setContinuous(true);

        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        finishTimer.stop();
        finishTimer.reset();
        isTimerStarted = false;

        initialDrivetrainAngle = drivetrain.getGyroAngle();
        angleErrorController.setSetpoint(initialDrivetrainAngle);
        angleErrorController.enable();

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(angle + drivetrain.getGyroAngle());
            drivetrain.getSwerveModule(i).zeroDistance();
            drivetrain.getSwerveModule(i).setTargetDistance(distance);
        }

        iterCount = 0;
        for (int i = 0; i < 4; i++) {
            try {
                // 191206 take space out of filename
                encPosLoggers[i] = Files.newBufferedWriter(Paths.get(String.format("/home/lvuser/encPos%d.csv", i)));
                encPosLoggers[i].write( "count, millisecs, abs(encoderPos), driveDist, gyroAngle, rotationFactor\n");
                encVelLoggers[i] = Files.newBufferedWriter(Paths.get(String.format("/home/lvuser/encVel%d.csv", i)));
            } catch (IOException e) {
                encPosLoggers[i] = null;
                encVelLoggers[i] = null;
            }
        }

        SmartDashboard.putNumber("Drive Distance Forward", distForward);
        SmartDashboard.putNumber("Drive Distance Right", distRight);
    }

    @Override
    protected void execute() {
        double forwardFactor = distForward / distance;
        double strafeFactor = -distRight / distance;

        double[] moduleAngles = drivetrain.calculateSwerveModuleAngles(forwardFactor, strafeFactor, rotationFactor);  // -rotationFactor); // 191206

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(moduleAngles[i]);
            
            try {
                // 191206 also save the time in milliseconds, and the driveDistance
                encPosLoggers[i].write( String.format("%d, %d, %f, %f, %f, %f\n",
                        iterCount, 
                        (int) System.currentTimeMillis(),
                        Math.abs(drivetrain.getSwerveModule(i).getDrivePosition()), 
                        drivetrain.getSwerveModule(i).getDriveDistance(),
                        drivetrain.getGyroAngle(),
                        rotationFactor));
                encVelLoggers[i].write(String.format("%d,%f\n",
                        iterCount,
                        Math.abs(drivetrain.getSwerveModule(i).getDriveVelocity())));

            } catch (IOException e) { }
            
        }
        iterCount++;
    }

    /**
     * This command finishes when all 4 of the the robot's wheels stay within TARGET_DISTANCE_BUFFER (inches)
     * of the desired drive distance for DISTANC_CHECK_TIME
     */
    @Override
    protected boolean isFinished() {
        boolean inBuffer = true;
        for (int i = 0; i < 4; i++) {
            // check that all wheels are within the TARGET_DISTANCE_BUFFER of the desired travel distance
            inBuffer &= Math.abs(distance - Math.abs(drivetrain.getSwerveModule(i).getDriveDistance())) < TARGET_DISTANCE_BUFFER;
        }

        if (inBuffer) {
            if (!isTimerStarted) {
                finishTimer.start();
                isTimerStarted = true;
            }
        } else {
            finishTimer.stop();
            finishTimer.reset();
            isTimerStarted = false;
        }

        return finishTimer.hasPeriodPassed(DISTANCE_CHECK_TIME);
    }

    @Override
    protected void end() {
        drivetrain.holonomicDrive(0, 0, 0);

        angleErrorController.disable();

        for (int i = 0; i < 4; i++) {
            try {
                if (encPosLoggers[i] != null)
                    encPosLoggers[i].close();
                if (encVelLoggers[i] != null)
                    encVelLoggers[i].close();
            } catch (IOException e) { }
            encPosLoggers[i] = null;
            encVelLoggers[i] = null;
        }
    }

    @Override
    protected void interrupted() {
        end();
    }
}