package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Claw {
    public enum State {
        CAUGHT,
        CATCHING,
        RELEASE,
        RELEASING,
        ERR
    }

    private Robot robot;
    private Sensors sensors;
    private Vision vision;
    private HardwareMap hardwareMap;

    private DcMotorEx stretchMotor;

    private ServoEx clawServo;


    public Claw(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        this.hardwareMap = hardwareMap;

        stretchMotor = hardwareMap.get(DcMotorEx.class, "RB");
        clawServo = new SimpleServo(
                hardwareMap, "clawServo", Constants.clawServoMinAngle, Constants.clawServoMaxAngle, AngleUnit.DEGREES
        );


    }

}
