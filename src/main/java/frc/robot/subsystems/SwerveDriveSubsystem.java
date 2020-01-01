package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import static frc.robot.RobotMap.*;

public class SwerveDriveSubsystem extends HolonomicDrivetrain {
    // set for SwerveyJr 191207
    public static final double WHEELBASE = 22;
    public static final double TRACKWIDTH = 19.5;
    public static final double WIDTH = 25.75;
    public static final double LENGTH = 28;

    private double angle_kP = 3.0;
    private double angle_kI = 0.0;
    private double angle_kD = 0.0;

	/*
	 * 0 is Front Left
	 * 1 is Front Right
	 * 2 is Back Right
	 * 3 is Back Left
	 */
    private SwerveDriveModule[] mSwerveModules;

    private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

    public SwerveDriveSubsystem() {
        super(WIDTH, LENGTH);
        zeroGyro();
        
        // 10/26/19 Big Switch from Talon to Spark Max controllers
        // 11/26/19 less positive angle offset settings turns wheel angle clockwise looking from the top
        //  more positive angle offset turns wheel counterclockwise looking from top
        mSwerveModules = new SwerveDriveModule[] {
            new SwerveDriveModule(0, 
                new CANSparkMax(DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                325.25 + 6), 

            new SwerveDriveModule(1, 
                new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless),
                68.421 + 10), 
            // 10/26/19 need to change the other 2 modules to SparkMax
            new SwerveDriveModule(2,
                new CANSparkMax(DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless),
                175.095 + 6),
            // 11/26/19 less positive angle offset settings turns wheel angle clockwise looking from the top
            new SwerveDriveModule(3,
                new CANSparkMax(DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                319.357 + 17),
            // 11/26/19 less positive angle offset settings turns wheel angle clockwise looking from the top   
        };

        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetAngle(0);
            module.setDriveGearRatio(5.7777);
            module.setDriveWheelRadius(module.getDriveWheelRadius() * 1.05);
            module.setMotionConstraints(getMaxAcceleration(), getMaxVelocity());
            module.setAngleKD(angle_kD);
            module.setAngleKI(angle_kI);
            module.setAngleKP(angle_kP);
        }
    }

    /**
     * compute the angles of the four modules and return a vector of them
     * uses private member isFieldOriented to decide to adjust based on gyro reading
     * @param forward how far forward the robot is going
     * @param strafe how far to the side the robot is going
     * @param rotation how much to spin the robot 
     * @param fo true if the computation is to be based on the field (field oriented)
     * @return vector of angles of the set points for the four modules, in order: LF, RF, RB, LB
     */
    public double[] calculateSwerveModuleAngles(double forward, double strafe, double rotation) {
        if (isFieldOriented()) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }

        double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
        double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
        double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
        double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

        return new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };
    }

    public AHRS getNavX() {
        return mNavX;
    }

    public double getGyroAngle() {
        double angle = mNavX.getAngle() - getAdjustmentAngle();
        angle %= 360;
        if (angle < 0) angle += 360;

        if (Robot.PRACTICE_BOT) {
            return angle;
        } else {
            return 360 - angle;
        }
    }

    public double getGyroRate() {
        return mNavX.getRate();
    }

    public double getRawGyroAngle() {
        double angle = mNavX.getAngle();
        angle %= 360;
        if (angle < 0) angle += 360;

        return angle;
    }

    public SwerveDriveModule getSwerveModule(int i) {
        return mSwerveModules[i];
    }

    @Override
    public void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented) {
        forward *= getSpeedMultiplier();
        strafe *= getSpeedMultiplier();
 
        if (fieldOriented) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) +
                    strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }
        
        double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
        double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
        double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
        double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

        double[] angles = new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };
        

        double[] speeds = new double[]{
                Math.sqrt(b * b + c * c),
                Math.sqrt(b * b + d * d),
                Math.sqrt(a * a + d * d),
                Math.sqrt(a * a + c * c)
        };

        double max = speeds[0];

        for (double speed : speeds) {
            if (speed > max) {
                max = speed;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (Math.abs(forward) > 0.05 ||
                    Math.abs(strafe) > 0.05 ||
                    Math.abs(rotation) > 0.05) {
                mSwerveModules[i].setTargetAngle(angles[i] + 180);
            } else {
                mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
            }
            mSwerveModules[i].setTargetSpeed(speeds[i]);
        }
    } 

    @Override
    public void stopDriveMotors() {
        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetSpeed(0);
        }
    }
    
    public void resetMotors() {
    	for(int i = 0; i < mSwerveModules.length; i++) {
    		mSwerveModules[i].resetMotor();
    	}
    }

    public SwerveDriveModule[] getSwerveModules() {
        return mSwerveModules;
    }

    @Override
    public double getMaxAcceleration() {
        return 5.5;
    }

    @Override
    public double getMaxVelocity() {
        return 10; 
    }

    public double getAngleKP() {
        return angle_kP;
    }
    public double getAngleKI() {
        return angle_kI;
    }
    public double getAngleKD() {
        return angle_kD;
    }
    public void setAngleKP( double k) {
        angle_kP = k;
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setAngleKP( k);
        }
    }
    public void setAngleKI( double k) {
        angle_kI = k;
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setAngleKI( k);
        }
    }
    public void setAngleKD( double k) {
        angle_kD = k;
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setAngleKD( k);
        }
    }

    /**
     * Setting all the modules to be brake or coast
     */
    public void setBrake(boolean b)
    {
        System.out.println( "setting brake mode to " + b);
        for (int i = 0; i < 4; i++)
        {
            mSwerveModules[i].setMotorBrake(b);
        }
    }

}
