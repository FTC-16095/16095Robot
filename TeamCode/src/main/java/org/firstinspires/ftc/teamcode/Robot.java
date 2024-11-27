package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {

    public final MecanumDrive drivetrain;
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
        drivetrain = new MecanumDrive(hardwareMap);

        Dashboard.setup();
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
        updateTelemetry();
    }

    private void updateSubsystems() {
        sensors.update();
        drivetrain.update();
        claw.clawUpdate();
        claw.stretchUpdate();
        elevator.update();

    }

    private void updateTelemetry() {
        // to be done
        Dashboard.packet.put("Loop Time", GET_LOOP_TIME());
        Dashboard.sendTelemetry();
    }

}