// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.HSVPreset;
import frc.robot.Constants.LEDColors;
import frc.robot.Constants.RGBPreset;

public class LED {
  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  private boolean idleState;

  private int pixelOffset = 0;
  
  private Timer timer;

  public LED() {
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(27);
    led.setLength(buffer.getLength());
    idleState = true;
    timer = new Timer();
    timer.start();
  }

  public void setIdleState(boolean state) {
    this.idleState = state;
  }

  public boolean getIdleState() {
    return idleState;
  }

  public void setColor(int r, int g, int b, double ... percent) {
    double p = 1;
    if (percent.length == 1) {
      p = MathUtil.clamp(percent[0], 0, 1);
    }

    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * p); i--) buffer.setRGB(i, r, g, b);
    led.setData(buffer);
    led.start();
  }

  public void setColor(RGBPreset l) {
    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * l.percent); i--) buffer.setRGB(i, l.r, l.g, l.b);
    led.setData(buffer);
    led.start();
  }

  public void setColor(RGBPreset l, double percent) {
    percent = MathUtil.clamp(percent, 0, 1);
    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * percent); i--) buffer.setRGB(i, l.r, l.g, l.b);
    led.setData(buffer);
    led.start();
  }

  public void cyanInPurpleStream(double time) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, 180, 255, 128);
      buffer.setHSV(pixelOffset, 90, 255, 128);
    }
    if (timer.get() < time) {
      if (pixelOffset == buffer.getLength()-1) {
        pixelOffset = 0;
      }
    } else {
      pixelOffset += 1;
      timer.reset();
    }
    led.setData(buffer);
    led.start();
  }

  public void setIdleColor() {
    for (int i = 0; i < buffer.getLength(); i++) {
      setColor(LEDColors.IDLE);
    }
  }

  public void clear() {
    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    led.setData(buffer);
    led.start();
  }

  public void idleBlink(double timeOn, double timeOff) {
    if (timer.get() < timeOn) {
      setIdleColor();
    } else if (timer.get() < (timeOn+timeOff)) {
      clear();
    } else {
      timer.reset();
    }
  }

  public void singeBlink(RGBPreset p, double timeOn, double timeOff) {
    if (timer.get() < timeOn) {
      setColor(p);
    } else if (timer.get() < (timeOn+timeOff)) {
      clear();
    } else {
      timer.reset();
    }
  }

  public void doubleBlink(RGBPreset p, RGBPreset l, double timeOn, double timeOff) {
    if (timer.get() < timeOn) {
      setColor(p);
    } else if (timer.get() < (timeOn+timeOff)) {
      setColor(l);
    } else {
      timer.reset();
    }
  }

  public void wave(HSVPreset color, double time) {
    for (var i = 0; i < buffer.getLength(); i++) {
      double v = 175 + Math.sin((double) (i - pixelOffset) / (double) buffer.getLength() * Math.PI * 2) * 75;
      buffer.setHSV(i, color.h, color.v, (int) v);
    }
    if (timer.get() < time) {
      if (pixelOffset == buffer.getLength()-1) {
        pixelOffset = 0;
      }
    } else {
      pixelOffset += 1;
      timer.reset();
    }
    led.setData(buffer);
    led.start();
  }
}