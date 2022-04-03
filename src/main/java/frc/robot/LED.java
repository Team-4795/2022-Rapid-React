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

    public RGBPreset(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public static final class HSVPreset {
    public int h;
    public int s;
    public int v;

    public HSVPreset(int h, int s, int v) {
      this.h = h;
      this.s = s;
      this.v = v;
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

  private void setColor(int r, int g, int b, boolean isHSV, double... percent) {
    double p = 1;

    if (percent.length == 1) p = MathUtil.clamp(percent[0], 0, 1);

    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * p); i--) {
      if (isHSV) {
        buffer.setHSV(i, r, g, b);
      } else {
        buffer.setRGB(i, r, g, b);
      }
    }

    led.setData(buffer);
    led.start();
  }

  public void setColor(RGBPreset color, double percent) {
    setColor(color.r, color.g, color.b, false, percent);
  }

  public void setColor(RGBPreset color) {
    setColor(color.r, color.g, color.b, false, 1);
  }

  public void setColor(HSVPreset color) {
    setColor(color.h, color.s, color.v, true, 1);
  }

  public void wave(HSVPreset color, double time) {
    for (var i = 0; i < buffer.getLength(); i++) {
      double v = 140 + Math.sin((i - pixelOffset) / (double) buffer.getLength() * Math.PI * 2) * 115;
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