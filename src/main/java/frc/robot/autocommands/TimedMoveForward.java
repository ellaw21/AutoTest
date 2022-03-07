// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class TimedMoveForward extends CommandBase {
  /** Creates a new TimedMoveForward. */

  private double timeToRun;
  private double endTime;
  private Timer timer = RobotContainer.moveTimer; 


  public TimedMoveForward(double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    timeToRun = t; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startTime = timer.get();
    endTime = startTime + timeToRun; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveBase.driveForwardAuto();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= endTime);
  }
}
