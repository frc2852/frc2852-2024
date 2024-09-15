package frc.robot;

import frc.robot.constants.Constants.OperatorConstant;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.TurnTest;
import frc.robot.util.swerve.SwerveUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

  private SendableChooser<Command> autoChooser;

  private final Drive drive;
  private final TurnTest turnTest;

  /**
   * Constructs the container for the robot. Subsystems and command mappings are
   * initialized here.
   */
  public RobotContainer() {

    // Start data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Port forwarding
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);

    // Initialize subsystems
    // drive = new Drive();
    drive = null;
    turnTest = new TurnTest();
    
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
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureDriverBindings() {
    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstant.DEAD_BAND)),
                -SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstant.DEAD_BAND)),
                -SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getRightX(), OperatorConstant.DEAD_BAND)),
                true, true),
            drive));
  }

  /**
   * Gets the command to run in autonomous mode.
   *
   * @return The autonomous command to run.`
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}