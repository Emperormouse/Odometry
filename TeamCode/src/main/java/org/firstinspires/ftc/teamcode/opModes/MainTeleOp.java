package org.firstinspires.ftc.teamcode.opModes;

import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import static org.firstinspires.ftc.teamcode.utility.Util.*;

@TeleOp(name = "MainTele")
public class MainTeleOp extends LinearOpMode {
    public void waitSeconds(double seconds) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < (seconds * 1000));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        DcMotor backRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        GoBildaPinpointDriver odo;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-90.0, -135.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        waitForStart();

        double targetAngle = 0;
        while (opModeIsActive()) {
            odo.update();
            double frPower = 0;
            double flPower = 0;
            double brPower = 0;
            double blPower = 0;

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double botRot = odo.getPositionRadians().r;
            double pRot = 1.0;

            if (abs(rx) >= 0.1) {
                frPower += rx;
                brPower += rx;
                flPower -= rx;
                blPower -= rx;
                targetAngle = botRot;
            }

            double diffRot = targetAngle - botRot;

            //ROTATION
            frPower += diffRot * pRot;
            brPower += diffRot * pRot;
            flPower -= diffRot * pRot;
            blPower -= diffRot * pRot;

            //X-DIRECTION
            frPower += y * cos(botRot);
            brPower += y * cos(botRot);
            flPower += y * cos(botRot);
            blPower += y * cos(botRot);
            
            frPower -= y * sin(botRot);
            brPower += y * sin(botRot);
            flPower += y * sin(botRot);
            blPower -= y * sin(botRot);

            //Y-DIRECTION
            frPower += x * sin(botRot);
            brPower += x * sin(botRot);
            flPower += x * sin(botRot);
            blPower += x * sin(botRot);

            frPower += x * cos(botRot);
            brPower -= x * cos(botRot);
            flPower -= x * cos(botRot);
            blPower += x * cos(botRot);

            double denominator = max(1, max(max(max(abs(frPower), abs(brPower)), abs(flPower)), abs(blPower)));

            frontLeft.setPower(flPower / denominator);
            frontRight.setPower(frPower / denominator);
            backLeft.setPower(blPower / denominator);
            backRight.setPower(brPower / denominator);

            telemetry.addData("RX: ", rx);
            telemetry.update();
        }
    }
}