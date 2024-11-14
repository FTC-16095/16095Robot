package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Elevator {
    public enum State {
        TOP,
        BOTTOM,
        ELEVATING,
        REPOSITIONING
    }

    private Robot robot;
    private Sensors sensors;
    private HardwareMap hardwareMap;
    private Vision vision;


}
