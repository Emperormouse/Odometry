package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.Pos;
import static java.lang.Math.*;

public class Drive {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public Pos targetPos;
    public GoBildaPinpointDriver odo;
    Telemetry telemetry;

    public Drive(HardwareMap hardwareMap, Telemetry telementary) {
        this.telemetry = telementary;

        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        backLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        backRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-90.0, -135.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        targetPos = new Pos(0, 0, 0);
    }

    public void frontLeftPower(double power) {
        frontLeft.setPower(power);
    }
    public void backLeftPower(double power) {
        backLeft.setPower(power);
    }
    public void frontRightPower(double power) {
        frontRight.setPower(power);
    }
    public void backRightPower(double power) {
        backRight.setPower(power);
    }

    public Pos getPos() {
        return odo.getPosition();
    }

    public void setPos(Pos targetPos) {
        this.targetPos = targetPos;
    }

    public boolean runPID() {
        odo.update();
        //Just rotational now
        Pos botPos = odo.getPosition();
        Pos diffPos = Pos.subtract(targetPos, botPos);
        double botRads = botPos.r * Math.PI / 180;
        double pRot = 0.07;
        double pForwards = 0.01;

        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        //ROTATION
        frPower += diffPos.r * pRot;
        brPower += diffPos.r * pRot;
        flPower -= diffPos.r * pRot;
        blPower -= diffPos.r * pRot;

        //X-DIRECTION
        frPower += diffPos.x * cos(botRads) * pForwards;
        brPower += diffPos.x * cos(botRads) * pForwards;
        flPower += diffPos.x * cos(botRads) * pForwards;
        blPower += diffPos.x * cos(botRads) * pForwards;

        frPower -= diffPos.x * sin(botRads) * pForwards;
        brPower += diffPos.x * sin(botRads) * pForwards;
        flPower += diffPos.x * sin(botRads) * pForwards;
        blPower -= diffPos.x * sin(botRads) * pForwards;

        //Y-DIRECTION
        frPower += diffPos.y * sin(botRads) * pForwards;
        brPower += diffPos.y * sin(botRads) * pForwards;
        flPower += diffPos.y * sin(botRads) * pForwards;
        blPower += diffPos.y * sin(botRads) * pForwards;

        frPower += diffPos.y * cos(botRads) * pForwards;
        brPower -= diffPos.y * cos(botRads) * pForwards;
        flPower -= diffPos.y * cos(botRads) * pForwards;
        blPower += diffPos.y * cos(botRads) * pForwards;

        double denominator = max(1, max(max(max(abs(frPower), abs(brPower)), abs(flPower)), abs(blPower)));

        frontLeft.setPower(flPower / denominator);
        frontRight.setPower(frPower / denominator);
        backLeft.setPower(blPower / denominator);
        backRight.setPower(brPower / denominator);

        return false;
    }

    public void setBrakes(boolean isOn) {
        if(isOn)
        {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

}

