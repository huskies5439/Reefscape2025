// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToHauteur extends Command {
  private Ascenseur ascenseur;
  private Poignet poignet;

  public GoToHauteur(Ascenseur ascenseur, Poignet poignet) {
    this.ascenseur = ascenseur;
    this.poignet = poignet;
    addRequirements(ascenseur, poignet);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ascenseur.resetPID();
    poignet.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ascenseur.setPID(ascenseur.getHauteurCible());
    poignet.setPID(poignet.getAngleCible());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ascenseur.stop();
    poignet.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ascenseur.atCible() && poignet.atCible();
  }
}
