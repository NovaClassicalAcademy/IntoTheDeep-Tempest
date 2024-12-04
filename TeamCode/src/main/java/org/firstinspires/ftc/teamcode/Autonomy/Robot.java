package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    private final DcMotor FrontLeft;
    private final DcMotor FrontRight;
    private final DcMotor BackLeft;
    private final DcMotor BackRight;
    private final DcMotor EncoderWheel;
    private final DcMotor LiftLeft;
    private final DcMotor LiftRight;
    private final Servo ServoDump;
    private final Servo ServoLeft;
    private final Servo ServoRight;
    private final Servo ServoHingeLeft;
    private final Servo ServoHingeRight;
//    private final GyroSensor Gyro;

    private final double LiftPower = 0.5;
    private final int StartingPosition;


    public Robot(HardwareMap hardwareMap) {
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
        ServoLeft = hardwareMap.get(Servo.class, "ServoClawLeft");
        ServoRight = hardwareMap.get(Servo.class, "ServoClawRight");
        ServoHingeLeft = hardwareMap.get(Servo.class, "HingeLeft");
        ServoHingeRight = hardwareMap.get(Servo.class, "HingeRight");

        // TODO: get the gyro sensor's name and update mapping
//        Gyro = hardwareMap.get(GyroSensor.class, "GyroSensor");

        StartingPosition = EncoderWheel.getCurrentPosition();
    }

    private double getDistance() {
        int ticksPerRotation = 2000;
        double Rotations = (double) (EncoderWheel.getCurrentPosition() - StartingPosition) / ticksPerRotation;
        double encoderWheelDiameter = 1.82;
        return Rotations * encoderWheelDiameter * Math.PI;
    }

    private double getLiftPosition() {
        return LiftRight.getCurrentPosition();
    }

//    public void Turn(double degreeChange) {
//        //TODO: is our gyro a part of ModernRoboticsI2cGyro?
//        double currentAngle = Gyro.getHeading();
//        double targetAngle = currentAngle + degreeChange;
//        //TODO: is this number accurate? Do we need to decrease?
//        double angleRange = 2;
//
//        while (Math.abs(targetAngle - currentAngle) > angleRange) {
//
//            if (targetAngle > currentAngle) {
//                //TODO: figure out if this is turning left or right---should be left
//                FrontLeft.setPower(-0.15);
//                FrontRight.setPower(0.15);
//                BackLeft.setPower(-0.15);
//                BackRight.setPower(0.15);
//            } else {
//                //TODO: do the opposite of above once direction is figured out----should be right
//                FrontLeft.setPower(0.15);
//                FrontRight.setPower(-0.15);
//                BackLeft.setPower(0.15);
//                BackRight.setPower(-0.15);
//            }
//
//            FrontLeft.setPower(0.0);
//            FrontRight.setPower(0.0);
//            BackLeft.setPower(0.0);
//            BackRight.setPower(0.0);
//        }
//    }

    public void Move(double moveDistance) {
        double currentDistance = getDistance();
        double targetDistance = currentDistance + moveDistance;
        double rangeDistance = 1.5;

        while (Math.abs(targetDistance - currentDistance) > rangeDistance) {

            //TODO: is this going backwards or forwards-- forwards
            if (targetDistance > currentDistance) {
                FrontLeft.setPower(-0.5);
                FrontRight.setPower(-0.5);
                BackLeft.setPower(-0.5);
                BackRight.setPower(-0.5);
            } else { //TODO: is this going backwards or forwards-- backwards might need to change value
                FrontLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackLeft.setPower(0.5);
                BackRight.setPower(0.5);
            }

            currentDistance = getDistance();
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
    public void Strafe(double strafeDistance) {
        double currentStrafePosition = getDistance();
        double targetStrafeDistance = currentStrafePosition + strafeDistance;
        double distanceRange = 5; //TODO: does this need to be adjusted?

        while (Math.abs(targetStrafeDistance - currentStrafePosition) > distanceRange) {
            //TODO: is this left or right--should be right
            if (targetStrafeDistance > currentStrafePosition){
            FrontLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackLeft.setPower(0.5);
            BackRight.setPower(-0.5);
            } else { //TODO: should be left
                FrontLeft.setPower(0.5);
                FrontRight.setPower(-0.5);
                BackLeft.setPower(-0.5);
                BackRight.setPower(0.5);
            }

            currentStrafePosition = getDistance();
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void LowerHinge(){
        //TODO: which is the lowest position? LOOK IN TELEOP
        }

    public void Claw() {
                //opened
                ServoLeft.setPosition(0);
                ServoRight.setPosition(1);
                //closed
                ServoLeft.setPosition(0.44);
                ServoRight.setPosition(0.56);
    }

    public void LiftHinge(){
        //TODO: which is the highest position? LOOK IN TELEOP
    }
    public void LowerLift(double lowerHeight) {
        double currentLiftPosition = getLiftPosition();
        double targetPosition = currentLiftPosition + lowerHeight;
        double heightRange = 3;

        while (Math.abs(targetPosition - currentLiftPosition) > heightRange) {
                LiftLeft.setPower(-LiftPower);
                LiftRight.setPower(-LiftPower);

                currentLiftPosition = getLiftPosition();
            }

        LiftLeft.setPower(0.0);
        LiftRight.setPower(0.0);
    }

    public void Dump () {
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
        double currentLiftPosition = getLiftPosition();
        double targetPosition = currentLiftPosition + highHeight;
        double heightRange = 6;

        while (Math.abs(targetPosition - currentLiftPosition) > heightRange) {
            LiftLeft.setPower(LiftPower);
            LiftRight.setPower(LiftPower);

            currentLiftPosition = getLiftPosition();
        }
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