package org.firstinspires.ftc.team16095;

import static org.firstinspires.ftc.team16095.utils16095.Globals.GET_LOOP_TIME;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team16095.sensors.Sensors;
import org.firstinspires.ftc.team16095.subsystems.claw.Claw;
import org.firstinspires.ftc.team16095.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.team16095.subsystems.elevator.Elevator;
import org.firstinspires.ftc.team16095.utils16095.Dashboard;
import org.firstinspires.ftc.team16095.vision.Vision;

public class Robot {

    public final Drivetrain drivetrain;
    public final Sensors sensors;

    public HardwareMap hardwareMap;
    //public final Deposit deposit;
    public final Vision vision;
    public final Elevator elevator;
    public final Claw claw;

    public Robot(HardwareMap hardwareMap, Vision vision) {
        this.vision = vision;

        claw = new Claw(hardwareMap, this);
        sensors = new Sensors();
        elevator = new Elevator(this, sensors, hardwareMap, vision);
        drivetrain = new Drivetrain(hardwareMap, this, sensors, vision);

        Dashboard.setup();
    }

    private void updateTelemetry() {
        // to be done
        Dashboard.packet.put("Loop Time", GET_LOOP_TIME());
        Dashboard.sendTelemetry();
    }

}