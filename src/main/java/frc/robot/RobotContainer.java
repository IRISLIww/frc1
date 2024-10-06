// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.otb.otb;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.indexer.indexer;

public class RobotContainer {
  public static final CommandXboxController controller = new CommandXboxController(0);
  private final otb otb = new otb();
  private final shooter shooter = new shooter();
  private final indexer indexer = new indexer();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
     controller.a().whileTrue(new InstantCommand(() -> otb.setMotionMagic(2)));
     controller.y().onTrue(new InstantCommand(() -> otb.runboth(45,2)));

     controller.b().onTrue(new InstantCommand(() -> indexer.runintake(3)));
     controller.x().onTrue(new InstantCommand(() -> indexer.runamp(3)));   
     
     controller.leftTrigger().whileTrue(new RunCommand(() -> shooter.setMotionMagic(2)));
     controller.rightTrigger().onTrue(new InstantCommand(() -> shooter.setVelocity(2, 2)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
   
  }
}
