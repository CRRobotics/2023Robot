package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int count;

  public LED() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
    count = 0;

    led.setData(ledBuffer);
    led.start();
  }

  public void showCone() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 255, 255, 127);
    }
    led.setData(ledBuffer);
  }

  public void showCube() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 100, 13, 255);
    }
    led.setData(ledBuffer);
  }

  public void showAuto() {
    System.out.println("led running");
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 255, 0);
    }
    led.setData(ledBuffer);
  }

  public void codeRedRobotics() {
    System.out.println("WHO ARE WE?");
    System.out.println("CODE RED!");

    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 255, 1, 1);
    }
    led.setData(ledBuffer);
  }
  public void turnOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  public void test() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 255, 0, 0);
    }
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // ledBuffer.setRGB(count, 202, 17, 17);
    // System.out.println(count + "set");
    // count = (count + 1) % (ledBuffer.getLength());
    // System.out.println(count + "reset");
    // ledBuffer.setRGB(count + 1, 0, 0, 0);
  }
}
