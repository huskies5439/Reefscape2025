// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AscenseurDefaut extends Command {
  /** Creates a new AscenseurDefaut. */
  Ascenseur ascenseur;
  Poignet poignet;
  public AscenseurDefaut(Ascenseur ascenseur, Poignet poignet) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ascenseur = ascenseur;
    this.poignet = poignet; 

    addRequirements(ascenseur);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (poignet.getAngle()<-30 && ascenseur.getPositionExterne() < 0.13){
       ascenseur.hold();
    }
    else{
      ascenseur.setPID(Hauteur.sol[0]);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ascenseur.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
