package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utility.Pos;

import java.util.ArrayList;

@Autonomous(name = "Test Odometry")
public class TestOdometry extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, telemetry);

        waitForStart();

        drive.movementSet(new Pos[]{
            new Pos(2500, 0, -360),
            new Pos(2500, 2500, -720),
            new Pos(0, 2500, -360),
            new Pos(0, 0, 0),
        });


        /*while(!isStopRequested()) {
            drive.runPID();

            Pos p = drive.getPos();
            Pos t = drive.targetPos;
            telemetry.addLine(String.format("X:%.2f Y:%.2f R:%.2f ", p.x, p.y, p.r));
            telemetry.addLine(String.format("X:%.2f Y:%.2f R:%.2f ", t.x, t.y, t.r));

            telemetry.update();
        }*/
    }
}