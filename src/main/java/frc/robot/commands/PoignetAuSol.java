// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Poignet;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoignetAuSol extends Command {
  /** Creates a new PoignetAuSol. */
  Poignet poignet;
  AlgueManip algueManip;

  public PoignetAuSol(Poignet poignet, AlgueManip algueManip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.poignet = poignet;
    this.algueManip = algueManip;
    addRequirements(poignet);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (poignet.getAngle() < 85){
      if (algueManip.isAlgue()){
        poignet.setPID(0);
      }
      else{
        poignet.setPID(Hauteur.sol[1]);
      }
    }
    else{
      poignet.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    poignet.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
