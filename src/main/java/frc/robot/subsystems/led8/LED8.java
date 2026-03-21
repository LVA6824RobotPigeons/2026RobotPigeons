package frc.robot.subsystems.led8;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.led8.Colors.ComplexColor;

public class LED8 {

    public CANdle candle;
    public LEDManager manager;

    public int startIndex;
    public int amountOfLeds;

    public LED8(CANdle candle, int startIndex, int amountOfLeds) {
        this.candle = candle;
        this.startIndex = startIndex;
        this.amountOfLeds = amountOfLeds;
        manager = new LEDManager();
    }

    public void addColor(ComplexColor complexColor, int zIndex) {
        if(!manager.has(complexColor,zIndex)) manager.add(complexColor,zIndex);
        if(manager.getCurrentColor() == complexColor) refresh();
    }
    public void removeColor(int zIndex) {
        ComplexColor previousTop = manager.getCurrentColor();
        if(manager.remove(zIndex) != null && previousTop != manager.getCurrentColor()) refresh();
    }
    public void refresh() {
        System.out.println("Refreshed");
        candle.clearAllAnimations();
        if(manager.getCurrentColor() != null) manager.getCurrentColor().tick(1000,this);
        else candle.setControl(new SolidColor(startIndex,startIndex+amountOfLeds-1).withColor(new RGBWColor(0,0,0)));
    }

    public void process(int refreshRate) {

        ComplexColor color = manager.getCurrentColor();
        if(color != null) color.tick(refreshRate,this);
        else candle.setControl(new SolidColor(startIndex,startIndex+amountOfLeds-1).withColor(new RGBWColor(0,0,0)));

    }




}
