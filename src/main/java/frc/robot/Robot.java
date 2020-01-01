package frc.robot;

import frc.robot.commands.ResetMotorsCommand;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.commands.autonomous.VisionLineUpWithCubeCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// SocketVision imports
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.SocketVisionSendWrapper;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	public static final boolean PRACTICE_BOT = false;

	public static final double FIELD_INFO_TIMEOUT = 5;

	public double modAngEncMax = 0;
	public double modAngEncMin = 100;

	private static OI mOI;
	private static SwerveDriveSubsystem swerveDriveSubsystem;
	//private static GathererSubsystem gathererSubsystem;

	//private final AutonomousChooser autoChooser = new AutonomousChooser();
	private Command autoCommand;
	private final SendableChooser<String> startPosChooser = new SendableChooser<>();	
	private final SendableChooser<String> loadStationChooser = new SendableChooser<>();
	
	private Timer autoTimer;

	//
	// Socket communications with the Vision Co-Processor, the UP Board
	// socket sender
	public final SocketVisionSendWrapper sender_ = new SocketVisionSendWrapper("10.59.33.255", 5800);
	// Socket receivers. One is needed for each port to read from
  public final SocketVisionWrapper rft_ = new SocketVisionWrapper("10.59.33.255", 5801);
	// Socket constants
	public static final boolean SHOW_DEBUG_VISION = false;

  // Constructor for the class. Call the superconstructor with
  // the Watchdog period (in seconds) that you want to use. Default is 0.02.
  public Robot(){
    super(0.05); // set watchdog to .05 seconds
  }

	public static OI getOI() {
		return mOI;
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		mOI = new OI(this);

		// create our subsystems
		swerveDriveSubsystem = new SwerveDriveSubsystem();
		swerveDriveSubsystem.setBrake(true);
		
    // initialize using the XBox controllers
		mOI.registerControls();
		
		// build the Chooser so we can tell the robot which starting position we're in
		startPosChooser.addOption("Vision Line Up With Cube", "L");
    startPosChooser.setDefaultOption("Drive For Distance", "C");
		// startPosChooser.addOption("Right", "R");
		startPosChooser.addOption("None", "N");
		// 'print' the Chooser to the dashboard
		SmartDashboard.putData("Start Position", startPosChooser);

		// build Chooser, what path to proceed to the loading station
		// loadStationChooser.setDefaultOption( "None", "N");
		// loadStationChooser.addOption( "DARK", "D");
		// loadStationChooser.addOption("LIGHT", "L");
		// 'print' the Chooser to the dashboard
		SmartDashboard.putData("Proceed to Loading Station", loadStationChooser);
		
		SmartDashboard.putData("Reset Motors", new ResetMotorsCommand(swerveDriveSubsystem));

		SmartDashboard.putNumber("Angle kP ", swerveDriveSubsystem.getAngleKP());
		SmartDashboard.putNumber("Angle kI ", swerveDriveSubsystem.getAngleKI());
    SmartDashboard.putNumber("Angle kD ", swerveDriveSubsystem.getAngleKD());
  }

    @Override
    public void robotPeriodic() {

    }

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
    // 
		for (int i = 0; i < 4; i++) {
			swerveDriveSubsystem.getSwerveModule(i).robotDisabledInit();
    }
    
		visionShutDown(); // 12/23 jh_vision: shut down the vision socket reader thread
		try {
			autoCommand.cancel();
		} catch (Exception e) {}
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * AutonomousInit
   * Called just before Autonomous is run the first time.
   * Primes subsystems for autonomous periodic; uses the Chooser to set the auto
   * command. Starts autonomous and the auto timer.
	 */
	@Override
	public void autonomousInit() {
		// open the socket connection to comminucate with the coprocessor, i.e. the UP Board
		socketVisionInit();

    // Instantiate the auto timer
    autoTimer = new Timer();
    
    // Prime the subsystems for auto (could be in a swerveDriveSubsystem.autoInit())
		swerveDriveSubsystem.setFieldOriented( true);
		swerveDriveSubsystem.setBrake(true);

    // Cancel any residual autonomous commands (or command groups)
    if(autoCommand != null) autoCommand.cancel();
    
    // Let the local autocommand be the final word in selecting autonomous, so disabledInit
    // can cancel it.
    switch(startPosChooser.getSelected()){ // this is a switch on the Strings in startPosChooser
      case "L":
        autoCommand = new VisionLineUpWithCubeCommand(this, rft_); // Pass in robot object (this) and vision scanner for PID
        break;
      case "C":
        autoCommand = new DriveForDistanceCommand(swerveDriveSubsystem, 25); // Distance in inches
        break;
      // Put other cases in here
      // Because CommandGroup extends Command, autoCommand can be set to a CommandGroup as well
      default:
        autoCommand = null;
    }
    
    // AutoTimer is not necessary, but autoCommand.start or scheduler.add(autocommand) should always be last
    autoTimer.start();
    if( autoCommand != null){
      autoCommand.start();
      SmartDashboard.putString("Autonomous Command Running:", autoCommand.toString());
    } else {
      SmartDashboard.putString("Autonomous Command Running:", "None");
    }
  }


	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
    Scheduler.getInstance().run();
	}

  /**
   * Sets up the robot for teleop mode:
   * Initialize socketvision, sets up subsystems, resets encoders.
   * Does NOT start any commands because subsystem defaults and buttons
   * will handle commands from here on out.
   */
	@Override
	public void teleopInit() {
		// open the socket connection to read/write the coprocessor, in case it wasn't already done in auto
    socketVisionInit();

    // Set up the drivetrain
    swerveDriveSubsystem.setBrake(false);
    swerveDriveSubsystem.setFieldOriented( true);

    /*********** Set up other subsystems here ************/

    // Cancel autonomous just in case
		if (autoCommand != null) autoCommand.cancel();

    // Reset the swerve modules' encoders
		for (int i = 0; i < 4; i++)
			swerveDriveSubsystem.getSwerveModule(i).zeroDistance();
    
    // We don't need to start any commands because subsystem defaults will run
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() { 
    Scheduler.getInstance().run();
  }

  /********* Put getters here (subsystems, objects, etc.) ************/

  // Return the drivetrain
	public SwerveDriveSubsystem getDrivetrain() {
		return swerveDriveSubsystem;
	}

  // Return the timer for autonomous
	public Timer getAutoTimer() {
		return autoTimer;
	}


	/**
	 * This method properly instantiates and initializes sockets to read from and write to
	 * coprocessors. This needs to be called before
	 * any of these objects could be used -- so don't use them during disabled mode. This should be called during autonomous
	 * and teleop init methods. For ease of access, these objects are global and instantiated through the main class.
	 */
	private void socketVisionInit() {
	  sender_.init();
    rft_.init();
	}

	/** 
	 * 12/23 jh_vision: shut down the vision socket reader thread
	 * This method properly shuts down all coprocessor related objects and joins them to the main thread
	 * to comply with FRC guidelines during disabled mode. DONT CHANGE A WORD!
	 */
	private void visionShutDown() {
    sender_.shutDown();
    rft_.shutDown();
	}
}
