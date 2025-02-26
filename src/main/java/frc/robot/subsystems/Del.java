// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DEL extends SubsystemBase {

  private AddressableLED del = new AddressableLED(1);
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(66);

  public DEL() {
    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
  }

  @Override
  public void periodic() {
  }

//////////////////////////////DEL

  public void couleur(Color color){
    LEDPattern solide = LEDPattern.solid(color);
    solide.applyTo(delBuffer);
    del.setData(delBuffer);
  }

  public void breathe(Color color){
    LEDPattern base = LEDPattern.solid(color);
    LEDPattern pattern = base.breathe(Seconds.of(2));
    pattern.applyTo(delBuffer);
    del.setData(delBuffer);
  }
}
