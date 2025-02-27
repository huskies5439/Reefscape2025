// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Del extends SubsystemBase {

  private AddressableLED del = new AddressableLED(1);
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(66);

  public Del() {
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
    LEDPattern pattern = base.breathe(Seconds.of(1.0));
    pattern.applyTo(delBuffer);
    del.setData(delBuffer);
  }

  public void rainbow(){
    LEDPattern base = LEDPattern.rainbow(255,255); 
    //LEDPattern pattern = base.blink(Seconds.of(2),Seconds.of(1)); 
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    pattern.applyTo(delBuffer); 
    del.setData(delBuffer); 
  }
}
