package frc.robot.subsystems;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public LED() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(58);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  public void showCone() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 255, 255, 127);
    }
  }

  public void showCube() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 100, 13, 255);
    }
  }

  public void showAuto() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void turnOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 0, 0);
    }
  }
}
