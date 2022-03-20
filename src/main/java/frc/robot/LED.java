// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LED {
  public static final class RGBPreset {
    public int r;
    public int g;
    public int b;
    public double percent;

    public RGBPreset(int r, int g, int b, double ... percent) {
      this.r = r;
      this.g = g;
      this.b = b;

      if (percent.length == 1) {
        this.percent = MathUtil.clamp(percent[0], 0, 1);
      } else {
        this.percent = 1;
      }
    }
  }

  public static final class HSVPreset {
    public int h;
    public int s;
    public int v;
    public double percent;

    public HSVPreset(int h, int s, int v, double ... percent) {
      this.h = h;
      this.s = s;
      this.v = v;

      if (percent.length == 1) {
        this.percent = MathUtil.clamp(percent[0], 0, 1);
      } else {
        this.percent = 1;
      }
    }
  }

  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  private int pixelOffset = 0;
  
  private Timer timer;

  public LED() {
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(27);
    led.setLength(buffer.getLength());
    timer = new Timer();
    timer.start();
  }

  public void setColor(int r, int g, int b, double ... percent) {
    double p = 1;
    if (percent.length == 1) p = MathUtil.clamp(percent[0], 0, 1);

    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * p); i--) buffer.setRGB(i, r, g, b);
    led.setData(buffer);
    led.start();
  }

  public void setColor(RGBPreset l) {
    setColor(l.r, l.g, l.b, l.percent);
  }

  public void setColor(RGBPreset l, double percent) {
    setColor(l.r, l.g, l.b, percent);
  }

  public void wave(HSVPreset color, double time) {
    for (var i = 0; i < buffer.getLength(); i++) {
      double v = 175 + Math.sin((i - pixelOffset) / (double) buffer.getLength() * Math.PI * 2) * 75;
      buffer.setHSV(i, color.h, color.v, (int) v);
    }
    if (timer.get() < time) {
      if (pixelOffset == buffer.getLength() - 1) {
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