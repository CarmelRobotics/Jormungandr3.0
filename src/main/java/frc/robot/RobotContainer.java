// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.generated.*;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Intake intake = new Intake();
  public CANdle led = new CANdle(50);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive.withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive forward
                                                                                            // with negative Y
                                                                                            // (forward)
              .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * 1.25) // Drive counterclockwise
                                                                                    // with negative X (left)
      ));

      this.m_driverController.leftTrigger().onTrue(this.intake.setState(IntakeState.INTAKING)).onFalse(this.intake.setState(IntakeState.STOW));
      this.m_driverController.rightTrigger().onTrue(this.intake.setState(IntakeState.SCORING)).onFalse(this.intake.setState(IntakeState.STOW));
     // this.m_driverController.rightBumper().onTrue(this.intake.setState(IntakeState.SCORE_BACK)).onFalse(this.intake.setState(IntakeState.STOW));
      this.m_driverController.leftBumper().onTrue(this.intake.setState(IntakeState.STATION_INTAKE)).onFalse(this.intake.setState(IntakeState.STOW));
      this.m_driverController.a().onTrue(this.intake.setState(IntakeState.CLIMBSTART)).onFalse(this.intake.setState(IntakeState.CLIMBFINAL));
      new Trigger(()->this.intake.hasCoral()).whileTrue(Commands.runOnce(()->this.led.setControl(new StrobeAnimation(0, 100).withColor(new RGBWColor(0,255,0))))).onFalse(Commands.runOnce(()->this.led.setControl(new SolidColor(0, 100).withColor(new RGBWColor(255, 0, 0)))));
      this.m_driverController.b().onTrue(this.intake.setState(IntakeState.OUTTAKING)).onFalse(this.intake.setState(IntakeState.STOW));
      this.m_driverController.x().onTrue(this.drivetrain.zeroGyro());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
