// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autocommands.basicAuto;
import frc.robot.commands.*; 
import frc.robot.subsystems.*; 
import edu.wpi.first.wpilibj.Joystick; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Timer;


//imports the pheonix products 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


//Spark max imports (to import, install vendor library online and put this link in https://software-metadata.revrobotics.com/REVLib.json)
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;


//Servo import
import edu.wpi.first.wpilibj.Servo; 




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  //declares driving motors
  public static WPI_TalonSRX leftFrontTalon = new WPI_TalonSRX(1);
  public static WPI_VictorSPX leftBackVictor = new WPI_VictorSPX(3);
  public static WPI_TalonSRX rightFrontTalon = new WPI_TalonSRX(2); 
  public static WPI_VictorSPX rightBackVictor = new WPI_VictorSPX(4);

 //declares system motors
 public static WPI_TalonSRX intakeTalon = new WPI_TalonSRX(5);

 //declares spark max
 public static CANSparkMax leftShooterSpark = new CANSparkMax(6, MotorType.kBrushless);
 public static CANSparkMax rightShooterSpark = new CANSparkMax(7, MotorType.kBrushless);

 public static CANSparkMax leftClimberSpark = new CANSparkMax(8,MotorType.kBrushless);
 public static CANSparkMax rightClimberSpark = new CANSparkMax(9,MotorType.kBrushless);

//Compressors
 public static Compressor robotCompressor;

 //Solenoids
 //public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid();

 //servos
 public static Servo leftShooterServo = new Servo(1); 
 public static Servo rightShooterServo = new Servo(2); 



 public static DriveBase driveBase; 
 public static DriveWithJoystick driveWithJoystick;
 public static ShooterBase shooterBase; 
 public static IntakeBase intakeBase; 
 public static ClimberBase climberBase; 
 public static ShooterHoodBase shooterHoodBase; 
 
 //declares joystickis
 public static Joystick leftJoystick;
 public static Joystick rightJoystick;
 public static Joystick logitech;
 
 //declare joystick button
 public static JoystickButton shootButton; 
 public static JoystickButton intakeButton; 
 public static JoystickButton climberButton; 
 public static JoystickButton hoodButtonUp; 
 public static JoystickButton hoodButtonDown; 
 public static JoystickButton retractHood; 
 public static POVButton incrementUp; 
 public static POVButton incrementDown; 

 //declares timer
 public static Timer moveTimer;
 
   /** The container for the robot. Contains subsystems, OI devices, and commands. */
   public RobotContainer() {
 
     leftJoystick = new Joystick (0);
     rightJoystick = new Joystick (1);
     logitech = new Joystick (2); 
 
     shootButton = new JoystickButton(logitech, 4);
     intakeButton = new JoystickButton(logitech, 1);
     climberButton = new JoystickButton(logitech, 3);
     hoodButtonUp = new JoystickButton(logitech,9);
     hoodButtonDown = new JoystickButton(logitech,10);
     retractHood = new JoystickButton(logitech, 8);
     incrementUp = new POVButton(logitech, 0);
     incrementDown = new POVButton(logitech, 180);

 
 
 
     driveBase = new DriveBase();
     driveWithJoystick = new DriveWithJoystick();
     CommandScheduler.getInstance().setDefaultCommand(driveBase, driveWithJoystick);
 
     shooterBase = new ShooterBase();
     intakeBase = new IntakeBase();
     climberBase = new ClimberBase(); 
     shooterHoodBase = new ShooterHoodBase();
 
 // decalres functions of buttons
 
     shootButton.whileHeld(new ShootBall());
     shootButton.whenReleased(new StopBall());

     intakeButton.whileHeld(new IntakeStart());
     intakeButton.whenReleased(new IntakeStop());

     climberButton.whileHeld(new ClimberStart());
     climberButton.whenReleased(new ClimberStop());

     hoodButtonUp.whenPressed(new ShooterHoodUp());
     hoodButtonDown.whenPressed(new ShooterHoodDown());
     retractHood.whenPressed(new RetractHood());
     incrementUp.whileHeld(new IncrementHoodUp());
     incrementDown.whileHeld(new IncrementHoodDown());

 
 
 
     // Configure the button bindings
     configureButtonBindings();
   }
 
   /**
    * Use this method to define your button->command mappings. Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
   private void configureButtonBindings() {}
 
   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
     // An ExampleCommand will run in autonomous
     return Robot.autoChooser.getSelected();
   }
 
   public static Joystick getRightJoystick(){
     return rightJoystick;
   }
 
   public static Joystick getLeftJoystick(){
     return leftJoystick;
   }

   public static Joystick getLogitech(){
     return logitech; 
   }
 
 }

 