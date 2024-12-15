package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Robot {

    private final OpMode MyOpmode;
    private final DcMotor FrontLeft;
    private final DcMotor FrontRight;
    private final DcMotor BackLeft;
    private final DcMotor BackRight;
//    private final DcMotor XEncoderWheel;
    private final DcMotor YEncoderWheel;
    private final DcMotor LiftLeft;
    private final DcMotor LiftRight;
    private final Servo ServoDump;
    private final Servo ServoLeft;
    private final Servo ServoRight;
    private final Servo ServoGrip;
    private final Servo ServoHingeLeft;
    private final Servo ServoHingeRight;
    private final IMU Gyro;
    private final double LiftPower = 0.5;
    private final int StartingYPosition;
//    private final int StartingXPosition;


    public Robot(OpMode opMode) {
        MyOpmode = opMode;
        MyOpmode.telemetry.addData("Status", "Initialized");
        YEncoderWheel = MyOpmode.hardwareMap.get(DcMotor.class, "EncoderWheel");
//        XEncoderWheel = MyOpmode.hardwareMap.get(DcMotor.class, "StrafeWheel");

        FrontLeft = MyOpmode.hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = MyOpmode.hardwareMap.get(DcMotor.class, "FrontRight");

        BackLeft = MyOpmode.hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = MyOpmode.hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftLeft = MyOpmode.hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = MyOpmode.hardwareMap.get(DcMotor.class, "LiftRight");

        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoDump = MyOpmode.hardwareMap.get(Servo.class, "ServoDump");
        ServoLeft = MyOpmode.hardwareMap.get(Servo.class, "ServoClawLeft");
        ServoRight = MyOpmode.hardwareMap.get(Servo.class, "ServoClawRight");
        ServoGrip = MyOpmode.hardwareMap.get(Servo.class, "ServoGrip");
        ServoHingeLeft = MyOpmode.hardwareMap.get(Servo.class, "HingeLeft");
        ServoHingeRight = MyOpmode.hardwareMap.get(Servo.class, "HingeRight");
        Gyro = MyOpmode.hardwareMap.get(IMU.class, "GyroSensor");

        StartingYPosition = YEncoderWheel.getCurrentPosition();
//        StartingXPosition = XEncoderWheel.getCurrentPosition();
    }

    /// returns forward and backward distance
    private double getYDistance() {
        int ticksPerRotation = 2000;
        double Rotations = (double) (YEncoderWheel.getCurrentPosition() - StartingYPosition) / ticksPerRotation;
        double encoderWheelDiameter = 1.82;
        return Rotations * encoderWheelDiameter * Math.PI;
    }

   /// returns left and right distance
    //not installed yet either
//    private double getXDistance() {
//        int ticksPerRotation = 2000;
//        double Rotations = (double) (XEncoderWheel.getCurrentPosition() - StartingXPosition)/ ticksPerRotation;
//        double strafeWheelDiameter = 1.82;
//        return Rotations * strafeWheelDiameter * Math.PI;
//    }

    public void Turn(double degreeChange) {
        //counterclockwise is negative (-180) and clockwise is positive (180)
        double currentAngle = Gyro.getRobotYawPitchRollAngles().getYaw();
        double targetAngle = currentAngle + degreeChange;
        //TODO: is this number accurate? Do we need to decrease?
        double angleRange = 2;

        Gyro.resetYaw();

        while (Math.abs(targetAngle - currentAngle) > angleRange) {

            if (targetAngle > currentAngle) {
                //this is left
                FrontLeft.setPower(-0.15);
                FrontRight.setPower(0.15);
                BackLeft.setPower(-0.15);
                BackRight.setPower(0.15);
            } else {
                //this is right
                FrontLeft.setPower(0.15);
                FrontRight.setPower(-0.15);
                BackLeft.setPower(0.15);
                BackRight.setPower(-0.15);
            }

            FrontLeft.setPower(0.0);
            FrontRight.setPower(0.0);

            MyOpmode.telemetry.addData("Angle", Gyro.getRobotYawPitchRollAngles());
            MyOpmode.telemetry.update();
        }
    }

    public void Move(double moveDistance) {
        double currentDistance = getYDistance();
        double targetDistance = currentDistance + moveDistance;
        double rangeDistance = 1.5;

        while (Math.abs(targetDistance - currentDistance) > rangeDistance) {

            //this is forwards
            if (targetDistance > currentDistance) {
                FrontLeft.setPower(-0.5);
                FrontRight.setPower(-0.5);
                BackLeft.setPower(-0.5);
                BackRight.setPower(-0.5);
            } else { //this is backwards; might need to change value
                FrontLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackLeft.setPower(0.5);
                BackRight.setPower(0.5);

                MyOpmode.telemetry.addData("Distance", getYDistance());
                MyOpmode.telemetry.update();
            }

            currentDistance = getYDistance();
        }

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);
    }

//        //NOTE: this if else determines the backward or forward movement
//        if (moveDistance == 0) {
//            return;
//        }
//        else if (moveDistance < 0){
//            frontLeft.setPower(0.0);
//            frontRight.setPower(0.0);
//            backLeft.setPower(0.0);
//            backRight.setPower(0.0);
//        }
//        else {
//            //TODO: is this going back or forward?
//            frontLeft.setPower(0.15);
//            frontRight.setPower(0.15);
//            backLeft.setPower(0.15);
//            backRight.setPower(0.15);
//        }
//
//        while (Math.abs(currentDistance - initialDistance) < Math.abs(moveDistance)){
//            sleep(100); //TODO: decrease if i need to sample more often to get more accurate read on movement
//            currentDistance = GetDistance();
//        }
//    public void Strafe(double strafeDistance) {
//        double currentStrafePosition = getXDistance();
//        double targetStrafeDistance = currentStrafePosition + strafeDistance;
//        double distanceRange = 5; //TODO: does this need to be adjusted?
//
//        while (Math.abs(targetStrafeDistance - currentStrafePosition) > distanceRange) {
//            //TODO: is this left or right--should be right
//            if (targetStrafeDistance > currentStrafePosition){
//            FrontLeft.setPower(-0.5);
//            FrontRight.setPower(0.5);
//            BackLeft.setPower(0.5);
//            BackRight.setPower(-0.5);
//            } else { //TODO: should be left
//            FrontLeft.setPower(0.5);
//            FrontRight.setPower(-0.5);
//            BackLeft.setPower(-0.5);
//            BackRight.setPower(0.5);
//
//            telemetry.addData("Distance", getXDistance());
//            }
//
//            currentStrafePosition = getXDistance();
//        }
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//    }

    public void LowerHinge(){
        ServoHingeLeft.setPosition(0.575);
        ServoHingeRight.setPosition(0.675);
        }
    public void LiftHinge() {
        ServoHingeLeft.setPosition(1.0);
        ServoHingeRight.setPosition(0.25);
    }

    public void Claw() {
        //opened
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        //closed
        ServoLeft.setPosition(0.44);
        ServoRight.setPosition(0.56);
    }

    public void Gripper() {
        double OpenClaw = ServoGrip.getPosition();
        double CloseClaw = ServoGrip.getPosition();

        //open
        ServoGrip.setPosition(1);
        //close
        ServoGrip.setPosition(0);

        MyOpmode.telemetry.addData("OpenClaw", OpenClaw);
        MyOpmode.telemetry.addData("CloseClose", CloseClaw);
        MyOpmode.telemetry.update();
    }

    public void LowerLift(double lowerHeight) {
        double currentLiftPosition = LiftRight.getCurrentPosition();
        double targetPosition = currentLiftPosition + lowerHeight;
        double heightRange = 3;

        while (Math.abs(targetPosition - currentLiftPosition) > heightRange) {

        LiftLeft.setPower(LiftPower);
        LiftRight.setPower(LiftPower);

        currentLiftPosition = LiftRight.getCurrentPosition();

        MyOpmode.telemetry.addData("Current Lift Position", currentLiftPosition);
        MyOpmode.telemetry.update();
        }

        LiftLeft.setPower(0.0);
        LiftRight.setPower(0.0);
    }

    public void Dump() {
        double currentDumpPosition = ServoDump.getPosition();
        double endPosition = 1;

        if (currentDumpPosition < endPosition){
            ServoDump.setPosition(1);
        }
//            double startPosition = ServoDump.getPosition();
//            double currentPosition = startPosition;
//
//            ServoDump.setPosition(0.25);
//
//            while (currentPosition < 0.24 || currentPosition > 0.26) {
//                sleep(100);
//                currentPosition = ServoDump.getPosition();
//            }
//
//            sleep(3000);
//            ServoDump.setPosition(startPosition);
//
//            while (currentPosition < startPosition - 0.01 || currentPosition > startPosition + 0.01) {
//                sleep(100);
//                currentPosition = ServoDump.getPosition();
            }

    public void LiftLift (double highHeight){
        double currentLiftPosition = LiftRight.getCurrentPosition();
        double targetPosition = currentLiftPosition + highHeight;
        double heightRange = 6;

        while (Math.abs(targetPosition - currentLiftPosition) > heightRange) {

            LiftLeft.setPower(-LiftPower);
            LiftRight.setPower(-LiftPower);

            currentLiftPosition = LiftRight.getCurrentPosition();

            MyOpmode.telemetry.addData("Current Lift Position", currentLiftPosition);
            MyOpmode.telemetry.update();
        }
    }

    public double GetLiftCurrentPosition (){
        return LiftRight.getCurrentPosition();
    }

//
//            int currentLiftPosition = LiftRight.getCurrentPosition();
//
//            LiftLeft.setPower(-LiftPower);
//            LiftRight.setPower(-LiftPower);
//
//            //NOTE; lift position is reversed, a number closer to zero is in the down position.
//            //a number with a negative value is extended position.
//            while (currentLiftPosition > -2940) {
//                telemetry.addData("LiftRightPosition:", currentLiftPosition);
//                telemetry.update();
//
//                sleep(100);
//                currentLiftPosition = LiftRight.getCurrentPosition();
//            }
//
//            LiftLeft.setPower(0);
//            LiftRight.setPower(0);{
    }