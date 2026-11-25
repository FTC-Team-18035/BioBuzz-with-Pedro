package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerPicker {
    private double up = .6685;
    private double down = .2487;
    public Servo flowerPickerA;
    public Servo flowerPickerB;

    public FlowerPicker (HardwareMap hardwaremap) {

    this.flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
    this.flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");
    }
    public void initialize(){
    flowerPickerA.setPosition(.5);  // set to safe midpoint position
    flowerPickerB.setPosition(.5);
    }

    public void raise() {
        flowerPickerA.setPosition(up);
        flowerPickerB.setPosition(1-up);  // adjust for reverse operation
    }
    public void lower() {
        flowerPickerA.setPosition(down);
        flowerPickerB.setPosition(1-down);  // adjust for reverse operation
    }
}
