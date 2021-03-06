// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeBase extends SubsystemBase {

  public static WPI_TalonSRX IntakeTalon = RobotContainer.intakeTalon;   
  /** Creates a new IntakeBase. */

  public IntakeBase() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeStart(){
    IntakeTalon.set(-0.75);
  }
  public void intakeStop(){
    IntakeTalon.set(0);
  }
}
