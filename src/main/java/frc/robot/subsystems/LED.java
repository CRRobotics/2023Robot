package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private static final Color RED = new Color(255, 0, 0);
  private static final Color WHITE = new Color(255, 255, 255);
  private static final Color YELLOW = new Color(255, 255, 127);
  private static final Color BLUE = new Color(100, 13, 255);
  private static final Color OFF = new Color(0, 0, 0);
  private int count;
  private int speed = 4;
  private int speedCount;

  public LED() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
    count = 0;
    speedCount = 0;

    led.setData(ledBuffer);
    led.start();
  }

  public void showCone() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, YELLOW);
    }
    led.setData(ledBuffer);
  }

  public void showCube() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, BLUE);
    }
    led.setData(ledBuffer);
  }

  public void showAuto() {
    System.out.println("led running");
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, WHITE);
    }
    led.setData(ledBuffer);
  }

  public void red() {
    // System.out.println("WHO ARE WE?");
    // System.out.println("CODE RED!");

    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, RED);
    }
    led.setData(ledBuffer);
  }


  public void turnOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, OFF);
    }
    led.setData(ledBuffer);
  }

  public void test() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, RED);
    }
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    scrollHead(count);
    if (speedCount < speed) {
      speedCount++;
    }
    else {
      speedCount = 0;
      count++;
    }

    count %= ledBuffer.getLength();
    led.setData(ledBuffer);
  }

  public void scrollHead(int i) {
    i %= ledBuffer.getLength(); // checks for overflow
    // System.out.println("set: " + i); // debug
    ledBuffer.setLED(i, RED); // set color
    i++; // increment

    i %= ledBuffer.getLength();
    // System.out.println("reset: " + i);
    ledBuffer.setLED(i, OFF);
    i++; // increment

    i %= ledBuffer.getLength();
    System.out.println("reset: " + i);
    ledBuffer.setLED(i, WHITE);
    i++; // increment

    i %= ledBuffer.getLength();
    System.out.println("reset: " + i);
    ledBuffer.setLED(i, OFF);
  }
}
