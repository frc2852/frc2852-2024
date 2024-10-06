package frc.robot;

import frc.robot.commands.PivotLoadPosition;
import frc.robot.commands.PivotShootPosition;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.StopShooterSequence;
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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private SendableChooser<Command> autoChooser;

  @SuppressWarnings("unused")
  private final SDSMK4ITuner sdsMK4ITuner;
  private final Drive drive;

  private final NoteTracker noteTracker = new NoteTracker();
  private final ShooterPivot shooterPivot = new ShooterPivot(noteTracker);
  private final Intake intake = new Intake(noteTracker);
  private final Shooter shooter = new Shooter(noteTracker);

  private final ShootSequence shootSequence = new ShootSequence(shooter, intake);
  private final StopShooterSequence stopShooterSequence = new StopShooterSequence(shooter, shooterPivot);
  private final Command pivotLoadPosition = new PivotLoadPosition(shooterPivot);
  private final Command pivotShootPosition = new PivotShootPosition(shooterPivot);

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
  }

  private void configureOperatorBindings() {

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

    // driverController A button
    driverController.a().onTrue(
        new InstantCommand(() -> {
          if (!shootSequence.isScheduled()) {
            shootSequence.schedule();
          }
        }));

    // driverController X button
    driverController.x().onTrue(
        new InstantCommand(() -> {
          if (!stopShooterSequence.isScheduled()) {
            stopShooterSequence.schedule();
          }
        }));

    // OperatorController A button for pivoting to load position
    operatorController.a().onTrue(
        new InstantCommand(() -> {
          if (!pivotLoadPosition.isScheduled()) {
            pivotLoadPosition.schedule();
          }
        }));

    // OperatorController B button for pivoting to shoot position
    operatorController.b().onTrue(
        new InstantCommand(() -> {
          if (!pivotShootPosition.isScheduled()) {
            pivotShootPosition.schedule();
          }
        }));

    // operatorController X button
    operatorController.x().whileTrue(new ReverseIntake(intake));
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
