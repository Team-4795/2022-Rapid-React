// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
import frc.robot.Constants.LEDColors;

public class LED extends SubsystemBase {
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

  private HSVPreset defaultColor;

  private RobotContainer robotContainer;

  public LED(RobotContainer rc) {
    robotContainer = rc;
    Alliance alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Red) {
      defaultColor = LEDColors.RED;
    } else {
      defaultColor = LEDColors.BLUE;
    }

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
    for (int i = buffer.getLength() - 1; i > buffer.getLength() - Math.round(buffer.getLength() * p)-1; i--) buffer.setRGB(i, r, g, b);
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

  public void stream(HSVPreset color, double time) {
    time = MathUtil.clamp(time, 0.025, time);
    double v = 0;
    for (var i = 0; i < buffer.getLength(); i++) {
      v = 243-9*(i+pixelOffset);
      buffer.setHSV(i, color.h, (int) v, color.v);
    }
    if (timer.get() < time) {
      if (pixelOffset > (243/9)+1) {
        pixelOffset = 0;
      }
    } else {
      pixelOffset += 1;
      timer.reset();
    }
    led.setData(buffer);
    led.start();
  }

  public void setDefaultAllianceColor() {
    wave(defaultColor, 0.05);
  }

  public void leds() {
    RobotStates.setState(RobotStates.IDLE);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED STATE", RobotStates.getState().toString());
    switch (RobotStates.getState()) {
      case SHOOTING:
      setColor(LEDColors.SHOOTING);
      break;
      case CHARGING:
      setColor(LEDColors.SHOOTER_CHARGING, robotContainer.superstructure.shooter.getFlywheelPercentage());
      break;
      case ONE_BALL:
      setColor(LEDColors.HAS_BALL, 0.5);
      break;
      case TWO_BALL:
      setColor(LEDColors.HAS_BALL, 1);
      break;
      case CLIMBING:
      stream(LEDColors.CLIMBING, 0.03);
      break;
      default:
      setDefaultAllianceColor();
      break;
    }
  }

}