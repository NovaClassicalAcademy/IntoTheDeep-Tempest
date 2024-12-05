
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ParkAuto", group = "Autonomous")

public class ParkAuto1 extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor EncoderWheel;

    int TicksPerRotation = 2000;
    double EncoderWheelDiameter = 1.82;
    int StartingPosition;

    Double Lift_power = 0.5;
    DcMotor LiftLeft;
    DcMotor LiftRight;
    Servo ServoDump;

    /* Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        EncoderWheel = hardwareMap.get(DcMotor.class, "EncoderWheel");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");

        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoDump = hardwareMap.get(Servo.class, "ServoDump");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        StartingPosition = EncoderWheel.getCurrentPosition();

        waitForStart();
        //get off wall
        frontLeft.setPower(-0.25);
        frontRight.setPower(0.25);
        BackLeft.setPower(0.25);
        BackRight.setPower(-0.25);
        sleep(500);

        //go forward
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);

        //park
        while (opModeIsActive()) {
            double Distance = getDistance();
            int LiftRightPosition = LiftRight.getCurrentPosition();

            telemetry.addData("Distance: ", Distance);
            telemetry.addData("LiftRightPosition:", LiftRightPosition);

            if (Distance <= -3.5) {
                //stop
                frontLeft.setPower(0);
                frontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);

            if (LiftRightPosition > -1250 && !BackLeft.isBusy() || !BackRight.isBusy() || !frontLeft.isBusy() || !frontRight.isBusy()) {
                LiftLeft.setPower(-Lift_power);
                LiftRight.setPower(-Lift_power);
                }
        }

            telemetry.update();
        }
    }



    private double getDistance() {
        double Rotations = (double) (EncoderWheel.getCurrentPosition() - StartingPosition) / TicksPerRotation;
        return Rotations * EncoderWheelDiameter * Math.PI;
    }
}