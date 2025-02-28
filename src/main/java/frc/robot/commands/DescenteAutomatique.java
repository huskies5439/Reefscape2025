// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class DescenteAutomatique extends Command {
  BasePilotable basePilotable;
  Ascenseur ascenseur;
  Poignet poignet;



  public DescenteAutomatique(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {
    this.basePilotable = basePilotable;
    this.ascenseur = ascenseur;
    this.poignet = poignet;

    addRequirements(ascenseur, poignet);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ascenseur.hold();
    poignet.hold();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return basePilotable.isLoinRecif();
  }
}
