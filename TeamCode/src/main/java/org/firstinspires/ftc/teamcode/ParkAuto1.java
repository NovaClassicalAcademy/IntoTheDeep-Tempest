
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "HighBucketAuto", group = "Autonomous")

public class ParkAuto1 extends LinearOpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
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

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        FrontLeft.setPower(0.6);
        FrontRight.setPower(-0.6);
        BackLeft.setPower(-0.6);
        BackRight.setPower(0.6);
        sleep(500);

        FrontLeft.setPower(0.2);
        FrontRight.setPower(-0.2);
        BackLeft.setPower(0.2);
        BackRight.setPower(-0.2);
        sleep(500);

        FrontLeft.setPower(0.35);
        FrontRight.setPower(-0.35);
        BackLeft.setPower(-0.35);
        BackRight.setPower(0.35);
        sleep(500);




        //go forward
        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);



        //park
        while (opModeIsActive()) {
            double Distance = getDistance();
            int LiftRightPosition = LiftRight.getCurrentPosition();

            telemetry.addData("Distance: ", Distance);
            telemetry.addData("LiftRightPosition:", LiftRightPosition);

            if (Distance <= -7) {
                //stop
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);

                if (LiftRightPosition > -1250 && !BackLeft.isBusy() || !BackRight.isBusy() || !FrontLeft.isBusy() || !FrontRight.isBusy()) {
                    LiftLeft.setPower(-Lift_power);
                    LiftRight.setPower(-Lift_power);
                }
                if (LiftRightPosition < -2940) {
                    ServoDump.setPosition(0.25);
                    sleep(1000);
                    
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