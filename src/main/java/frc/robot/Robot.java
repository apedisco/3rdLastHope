// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SlewRateLimiter m_SpeedLimiter = new SlewRateLimiter(0); 
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0);

  private final Joystick m_IntakeJoystick = new Joystick(0); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
 
  private double start_angle;

  private double MaxSpeed = 2; // 6 meters per second desired top speed
  private double MaxAngularRate = .5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final CommandJoystick joystick = new CommandJoystick(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.5) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private  Telemetry logger;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    

    logger = new Telemetry(MaxSpeed);
    drivetrain.registerTelemetry(logger::telemeterize);
    start_angle = drivetrain.getRotation3d().getZ();


    // final JoystickButton IntakeButton = new JoystickButton(m_IntakeJoystick, 1);
    // final JoystickButton ShootingButton = new JoystickButton(m_IntakeJoystick, 2);
    // final JoystickButton ShooterReverseButon = new JoystickButton(m_IntakeJoystick, 3);
    // final JoystickButton AutoButton = new JoystickButton(m_IntakeJoystick, 4);
   
    // ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem));
    // IntakeButton.whileTrue(new IntakeCommand(m_IntakeSubsystem));
    // ShootingButton.whileTrue(new ShootingCommand(m_IntakeSubsystem));
    // (IntakeButton.and(ShootingButton)).whileTrue(new shootandstage(m_IntakeSubsystem)); //composed command for multiple commands
    // AutoButton.onTrue(new AutoTest(drivetrain));
    
    
   
    // ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem));
    


    //IntakeButton.whileTrue(new IntakeCommand(m_IntakeSubsystem));
    //ShootingButton.whileTrue(new ShootingCommand(m_IntakeSubsystem));
    //(IntakeButton.and(ShootingButton)).whileTrue(new shootandstage(m_IntakeSubsystem)); //composed command for multiple commands
    //AutoButton.onTrue(new AutoTest(drivetrain));
    
    
   
    
    //configureBindings();

  }



  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
            .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        drivetrain.moveLinear(.2, 0, drivetrain, drive);
        break;
      case kDefaultAuto:
      default:
        System.out.println("Running Auto");
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_IntakeJoystick.getRawButtonPressed(1)){
      m_IntakeSubsystem.IntakeIn();
    }
    if (m_IntakeJoystick.getRawButtonReleased(1)){
      m_IntakeSubsystem.IntakeOff();
    }

    if (m_IntakeJoystick.getRawButtonPressed(2)){
      m_IntakeSubsystem.StagingIn();
    }
    if (m_IntakeJoystick.getRawButtonReleased(2)){
      m_IntakeSubsystem.StagingOff();
    }

     drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
            .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
