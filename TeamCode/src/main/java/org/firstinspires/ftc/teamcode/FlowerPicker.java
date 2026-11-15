package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerPicker {
    private double up = .6;  // TODO - correct value after testing
    private double down = .45;  // TODO - correct value after testing
    private Servo flowerPickerA;
    private Servo flowerPickerB;

    public FlowerPicker (HardwareMap hardwaremap) {

    this.flowerPickerA = hardwaremap.get(Servo .class,"flowerPickerA");
    this.flowerPickerB = hardwaremap.get(Servo .class,"flowerPickerB");

    flowerPickerA.setPosition(.5);  // set to safe midpoint position
    flowerPickerB.setPosition(.5);
    }

    public void raise() {
        flowerPickerA.setPosition(up);
        flowerPickerA.setPosition(up);
    }
    public void lower() {
        flowerPickerA.setPosition(down);
        flowerPickerA.setPosition(down);
    }
}
