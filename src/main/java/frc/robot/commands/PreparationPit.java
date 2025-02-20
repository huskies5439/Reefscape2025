// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class PreparationPit extends SequentialCommandGroup {

  public PreparationPit(Ascenseur ascenseur, Poignet poignet) {
    
    addCommands(
    //Débarre le servo moteur de l'ascenseur
    Commands.runOnce(ascenseur::debarrer),

    //monte l'ascenseur jusqu'a ce que la limiteSwitch soit désengager
    Commands.run(()->ascenseur.setVoltage(1)).until(()-> {
        return !ascenseur.isLimitSwitch();
      }
    ),

    //Monter un peu plus
    new WaitCommand(0.5),

    //dessend l'ascenseur jusqu'a ce que la limiteSwitch soit engager
    Commands.run(()->ascenseur.setVoltage(-0.5),ascenseur).until(ascenseur::isLimitSwitch),

    //arrête l'ascenseur
    Commands.runOnce(ascenseur :: stop), 
    Commands.runOnce(ascenseur :: resetEncodeurExterne),

    //monte l'ascenseur une deuxieme fois moins loins
    Commands.run(()->ascenseur.setVoltage(1), ascenseur).withTimeout(0.2),

    //descend l'ascenseur jusqu'a ce que la limiteSwitch soit engager
    Commands.run(()->ascenseur.setVoltage(-0.15),ascenseur).until(ascenseur::isLimitSwitch),

    Commands.runOnce(ascenseur :: stop), 
    Commands.runOnce(ascenseur :: resetEncodeurExterne),

    //définie la position de l'ascenseur a 20cm
    Commands.run(()->ascenseur.setVoltage(2),ascenseur).until(()->{
        return ascenseur.getPositionExterne() >= 0.2;
      }
    ),

    new ParallelRaceGroup(
      Commands.run(ascenseur::hold),
      new SequentialCommandGroup(
        // descends le pognet jusqu'à l'activation de la limitswitch
        Commands.run(()->poignet.setVoltage(-1),poignet).until(poignet::isLimitSwitch),

        Commands.runOnce(poignet :: stop),
        Commands.runOnce(poignet :: resetEncodeurLimitSwitch),

        // remonte légèrement le poignet
        Commands.run(()->poignet.setVoltage(1)).withTimeout(0.5),
  
        // descends le poignet jusqu'à l'activation de la limitswitch
        Commands.run(() ->poignet.setVoltage(-1),poignet).until(poignet::isLimitSwitch),
    
        Commands.runOnce(poignet :: stop), 
        Commands.runOnce(poignet :: resetEncodeurLimitSwitch),
      
        Commands.runEnd(()->poignet.setVoltage(1), poignet::stop, poignet).until(()->{
            return poignet.getAngle() >= 80;
          })
    )),

    //reHome l'ascenseur
    Commands.runEnd(ascenseur::descendre, ascenseur::stop, ascenseur).until(ascenseur :: isLimitSwitch)
    );
  }
}
