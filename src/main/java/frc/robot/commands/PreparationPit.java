// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;


public class PreparationPit extends SequentialCommandGroup {
  
  public PreparationPit(Ascenseur ascenseur, Poignet poignet) {
    
    addCommands(

    Commands.run(()->ascenseur.setVoltage(1), ascenseur).withTimeout(0.5),
    Commands.run(()->ascenseur.setVoltage(-0.5),ascenseur).until(ascenseur::isLimitSwitch),
    Commands.runOnce(ascenseur :: stop), 
    Commands.runOnce(ascenseur :: resetEncodeurExterne),

    Commands.run(()->poignet.setVoltage(-0.5),poignet).until(poignet::isHome),
    Commands.runOnce(poignet :: stop),
    Commands.runOnce(poignet :: resetEncoders)
   
    );
  }
}
