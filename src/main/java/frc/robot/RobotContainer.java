package frc.robot;

import frc.robot.constants.Constants.ConfigurationProperties;
import frc.robot.constants.Constants.OperatorConstant;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SDSMK4ITuner;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.util.NoteTracker;
import frc.robot.util.swerve.SwerveUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer is the class where the bulk of the robot's systems are
 * declared.
 * Here, subsystems, OI devices, and commands are set up and should be the only
 * place that this configuration exists in the code.
 */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final CommandXboxController sysIdController;

  private SendableChooser<Command> autoChooser;

  @SuppressWarnings("unused")
  private final SDSMK4ITuner sdsMK4ITuner;
  private final Drive drive;

  private final NoteTracker noteTracker = new NoteTracker();
  private final ShooterPivot shooterPivot = new ShooterPivot();
  private final Intake intake = new Intake(noteTracker);
  private final Shooter shooter = new Shooter(noteTracker);

  /**
   * Constructs the container for the robot. Subsystems and command mappings are
   * initialized here.
   */
  public RobotContainer() {

    // Start data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OperatorConstant.OPERATOR_CONTROLLER_PORT);
    sysIdController = ConfigurationProperties.SYS_ID ? new CommandXboxController(OperatorConstant.SYSID_CONTROLLER_PORT) : null;

    // Initialize subsystems
    if (ConfigurationProperties.SWERVE_TUNE) {
      drive = null;
      sdsMK4ITuner = new SDSMK4ITuner();
    } else {
      drive = new Drive();
      sdsMK4ITuner = null;
    }

    // Configuration
    configureBindings();
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureBindings() {
    if (drive != null) {
      configureDriverBindings();
    }

    configureOperatorBindings();

    if (ConfigurationProperties.SYS_ID) {
      configureSysIdBindings();
    }
  }

  /**
   * Configures the driver bindings for normal operation.
   */
  private void configureDriverBindings() {
    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstant.DEAD_BAND)),
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstant.DEAD_BAND)),
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getRightX(), OperatorConstant.DEAD_BAND)),
                true, true),
            drive));
  }

  /**
   * Configures the operator bindings for normal operation.
   */
  private void configureOperatorBindings() {
  }

  /**
   * Configures the shooter for SysId data collection.
   */
  private void configureSysIdBindings() {
    // Create a thread to handle SysId data communication
    new Thread(() -> {
      // NetworkTable entries for SysId
      var table = NetworkTableInstance.getDefault().getTable("FlywheelSysId");
      var voltageEntry = table.getEntry("voltage");
      var timestampEntry = table.getEntry("timestamp");
      var velocityEntry = table.getEntry("velocity");
      var ackEntry = table.getEntry("ackNumber");

      MedianFilter velocityFilter = new MedianFilter(5);
      int prevAck = -1;

      while (!Thread.interrupted()) {
        // Get the voltage command from SysId
        double voltage = voltageEntry.getDouble(0.0);

        // Apply the voltage to the shooter
        shooter.setSysIdVoltageCommand(voltage);

        // Get the current timestamp
        double timestamp = Timer.getFPGATimestamp();

        // Get the current shooter velocity
        double velocity = shooter.getShooterVelocity();

        // Optionally filter the velocity to reduce noise
        double filteredVelocity = velocityFilter.calculate(velocity);

        // Send data back to SysId
        timestampEntry.setDouble(timestamp);
        velocityEntry.setDouble(filteredVelocity);

        // Handle acknowledgment number to synchronize data
        int ackNumber = (int) ackEntry.getDouble(0);
        if (ackNumber != prevAck) {
          prevAck = ackNumber;
          // Reset filters or perform any necessary actions on new data
          velocityFilter.reset();
        }

        // Sleep for a short period to avoid hogging CPU
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          break;
        }
      }
    }, "SysId-Thread").start();
  }

  /**
   * Gets the command to run in autonomous mode.
   *
   * @return The autonomous command to run.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
