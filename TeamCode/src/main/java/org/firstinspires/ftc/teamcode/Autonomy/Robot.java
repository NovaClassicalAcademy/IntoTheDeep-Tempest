package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    private final OpMode MyOpmode;
    private final DcMotor FrontLeft;
    private final DcMotor FrontRight;
    private final DcMotor BackLeft;
    private final DcMotor BackRight;
    private final DcMotor YEncoderWheel;
    private final DcMotor LiftLeft;
    private final DcMotor LiftRight;
    private final Servo ServoDump;
    private final Servo ServoLeft;
    private final Servo ServoRight;
    private final Servo ServoGrip;
    private final Servo ServoHingeLeft;
    private final Servo ServoHingeRight;
    private final IMU MyGyro;
    public final double LiftPower = 0.5;
    private final int StartingYPosition;
    private double LiftOffset;


    public Robot(OpMode opMode) {
        MyOpmode = opMode;
        MyOpmode.telemetry.addData("Status", "Initialized");

        YEncoderWheel = MyOpmode.hardwareMap.get(DcMotor.class, "EncoderWheel");

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

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftLeft = MyOpmode.hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = MyOpmode.hardwareMap.get(DcMotor.class, "LiftRight");

        LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // NOTE: Reset encoder.
        LiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ServoDump = MyOpmode.hardwareMap.get(Servo.class, "ServoDump");
        ServoLeft = MyOpmode.hardwareMap.get(Servo.class, "ServoClawLeft");
        ServoRight = MyOpmode.hardwareMap.get(Servo.class, "ServoClawRight");
        ServoGrip = MyOpmode.hardwareMap.get(Servo.class, "ServoGrip");
        ServoHingeLeft = MyOpmode.hardwareMap.get(Servo.class, "HingeLeft");
        ServoHingeRight = MyOpmode.hardwareMap.get(Servo.class, "HingeRight");
        MyGyro = MyOpmode.hardwareMap.get(IMU.class, "GyroSensor");

        StartingYPosition = YEncoderWheel.getCurrentPosition();
    }

    /// returns forward and backward distance
    private double GetYEncoderPosition() {
        int ticksPerRotation = 2000;
        double Rotations = (double) (YEncoderWheel.getCurrentPosition() - StartingYPosition) / ticksPerRotation;
        double encoderWheelDiameter = 1.82;
        return Rotations * encoderWheelDiameter * Math.PI;
    }

//    public void Turn(double degreeChange) {
//        //counterclockwise is negative (-180) and clockwise is positive (180)
//        MyGyro.resetYaw();
//        double currentAngle = MyGyro.getRobotYawPitchRollAngles().getYaw();
//        double angleRange = 0.5;
//
//        if (degreeChange < 0) {
//            //this is right
//            FrontLeft.setPower(0.45);
//            FrontRight.setPower(-0.45);
//            BackLeft.setPower(0.45);
//            BackRight.setPower(-0.45);
//        } else if (degreeChange > 0) {
//            //this is left
//            FrontLeft.setPower(-0.45);
//            FrontRight.setPower(0.45);
//            BackLeft.setPower(-0.45);
//            BackRight.setPower(0.45);
//        }
//
//        while (Math.abs(degreeChange) - Math.abs(currentAngle) > angleRange) {
//
//            currentAngle = MyGyro.getRobotYawPitchRollAngles().getYaw();
//
//            MyOpmode.telemetry.addData("Angle", MyGyro.getRobotYawPitchRollAngles());
//            MyOpmode.telemetry.addData("Yaw", currentAngle);
//            MyOpmode.telemetry.update();
//        }
//
//        FrontLeft.setPower(0.0);
//        FrontRight.setPower(0.0);
//        BackLeft.setPower(0.0);
//        BackRight.setPower(0.0);
//    }

//    /// HINGE
//    public void LowerHinge() {
//        double currentHingePosition = ServoHingeLeft.getPosition();
//
//        ServoHingeLeft.setPosition(0.675);
//        ServoHingeRight.setPosition(0.575);
//
//        while (currentHingePosition != 0.675) {
//            currentHingePosition = ServoHingeLeft.getPosition();
//
//            MyOpmode.telemetry.addData("Hinge Rotation", currentHingePosition);
//            MyOpmode.telemetry.update();
//        }
//    }
//
//    public void LiftHinge() {
//        double currentHingePosition = ServoHingeLeft.getPosition();
//
//        ServoHingeLeft.setPosition(1.0);
//        ServoHingeRight.setPosition(0.25);
//
//        while (currentHingePosition != 1.0) {
//            currentHingePosition = ServoHingeLeft.getPosition();
//
//            MyOpmode.telemetry.addData("Hinge Rotation", currentHingePosition);
//            MyOpmode.telemetry.update();
//        }
//    }
//
//    / CLAW
//    public void OpenClaw() {
//        ServoLeft.setPosition(0.0);
//        ServoRight.setPosition(1.0);
//
//        while (ServoLeft.getPosition() != 0.0 && ServoRight.getPosition() != 1.0) {
//
//            MyOpmode.telemetry.addData("Left Claw Rotation", ServoLeft.getPosition());
//            MyOpmode.telemetry.addData("Right Claw Rotation", ServoRight.getPosition());
//            MyOpmode.telemetry.update();
//        }
//    }
//
//    public void CloseClaw() {
//        ServoLeft.setPosition(0.44);
//        ServoRight.setPosition(0.56);
//
//        while (ServoLeft.getPosition() != 0.44 && ServoRight.getPosition() != 0.56) {
//
//            MyOpmode.telemetry.addData("Left Claw Rotation", ServoLeft.getPosition());
//            MyOpmode.telemetry.addData("Right Claw Rotation", ServoRight.getPosition());
//            MyOpmode.telemetry.update();
//        }
//    }
//
//    /// GRIPPER
//    public void OpenGripper() {
//        ServoGrip.setPosition(0.15);
//
//        while (ServoGrip.getPosition() != 0.15) {
//
//            MyOpmode.telemetry.addData("Grip Rotation", ServoGrip.getPosition());
//            MyOpmode.telemetry.update();
//        }
//    }
//
//    public void CloseGripper() {
//        ServoGrip.setPosition(0.8);
//
//        while (ServoGrip.getPosition() != 0.8) {
//
//            MyOpmode.telemetry.addData("Grip Rotation", ServoGrip.getPosition());
//            MyOpmode.telemetry.update();
//        }
//    }
////
////    /// DUMP
////    public void Dump() {
////        double endPosition = 0.25;
////
////        ServoDump.setPosition(endPosition);
////
////        while (ServoDump.getPosition() != endPosition) {
////
////            MyOpmode.telemetry.addData("Dump Rotation", ServoDump.getPosition());
////            MyOpmode.telemetry.update();
////        }
////    }
////
////    public void RetractDump() {
////        double endPosition = 1;
////
////        ServoDump.setPosition(endPosition);
////
////        while (ServoDump.getPosition() != endPosition) {
////
////            MyOpmode.telemetry.addData("Retract Rotation", ServoDump.getPosition());
////            MyOpmode.telemetry.update();
////        }
////    }
////}
////
////    /// LIFT, make sure max is long enough to do function
//////    public void ExtendLift(double targetPosition, double maxTimeInSeconds) {
//////        LiftOffset = ResetLiftToZero();
//////
////////        targetPosition =- LiftOffset;
//////        double currPosition = LiftLeft.getCurrentPosition() * -1;
//////        double posTolerance = 1;
//////        double liftPower = -LiftPower;
//////
//////        double startTime = MyOpmode.getRuntime();
//////
//////        LiftLeft.setPower(liftPower);
//////        LiftRight.setPower(liftPower);
//////
//////        while (currPosition < (targetPosition - posTolerance) || currPosition > (targetPosition + posTolerance)) {
//////            currPosition = LiftLeft.getCurrentPosition() * -1;
//////            double elapsedTime = MyOpmode.getRuntime() - startTime;
//////            MyOpmode.telemetry.addData("Elapsed", elapsedTime);
//////
//////            if (elapsedTime >= maxTimeInSeconds) {
//////                MyOpmode.telemetry.addData("Error", "Lift time out");
//////                break;
//////            }
//////
//////            MyOpmode.telemetry.addData("Current Lift Position", currPosition);
//////            MyOpmode.telemetry.update();
//////            MyOpmode.telemetry.addData("Target Position", targetPosition);
//////            MyOpmode.telemetry.update();
//////
//////        }
//////
//////        LiftLeft.setPower(0.01);
//////        LiftRight.setPower(0.01);
//////    }
//////
//////        public void LowerLift (double targetPosition, double maxTimeInSeconds){
//////            targetPosition =- LiftOffset;
//////            double currPosition = LiftLeft.getCurrentPosition() * -1;
//////            double posTolerance = 1;
//////            double liftPower = LiftPower;
//////
//////            LiftLeft.setPower(liftPower);
//////            LiftRight.setPower(liftPower);
//////
//////            double startTime = MyOpmode.getRuntime();
//////
//////            while (currPosition < (targetPosition - posTolerance) || currPosition > (targetPosition + posTolerance)) {
//////                currPosition = LiftLeft.getCurrentPosition() * -1;
//////
//////                double elapsedTime = MyOpmode.getRuntime() - startTime;
//////                MyOpmode.telemetry.addData("Elapsed", elapsedTime);
//////
//////                if (elapsedTime >= maxTimeInSeconds) {
//////                    MyOpmode.telemetry.addData("Error", "Lift time out");
//////                    break;
//////                }
//////
//////                MyOpmode.telemetry.addData("Current Lift Position", currPosition);
//////                MyOpmode.telemetry.update();
//////                MyOpmode.telemetry.addData("Target Position", targetPosition);
//////                MyOpmode.telemetry.update();
//////            }
//////
//////            LiftLeft.setPower(0);
//////            LiftRight.setPower(0);
//////        }
////    public void MoveLift(double targetPosition,double maxTimeInSeconds) {
////        double currentLiftPosition = LiftLeft.getCurrentPosition()* -1;
////        double heightRange = 1;
////        double liftPower = -LiftPower;
////
////        if (targetPosition < currentLiftPosition){
////            liftPower = LiftPower;
////        }
////
////        double startTime = MyOpmode.getRuntime();
////
////        while (Math.abs(currentLiftPosition - targetPosition) > heightRange) {
////            double elapsedTime = MyOpmode.getRuntime()-startTime;
////            MyOpmode.telemetry.addData("Elapsed", elapsedTime);
////
////            if (elapsedTime >= maxTimeInSeconds){
////                MyOpmode.telemetry.addData("Error", "Lift time out");
////                break;
////            }
////
////            LiftLeft.setPower(liftPower);
////            LiftRight.setPower(liftPower);
////
////            currentLiftPosition = LiftLeft.getCurrentPosition()* -1;
////
////            MyOpmode.telemetry.addData("Current Lift Position", currentLiftPosition);
////            MyOpmode.telemetry.update();
////            MyOpmode.telemetry.addData("Target Position", targetPosition);
////            MyOpmode.telemetry.update();
////        }
////
////        //Once it is in range
////        LiftLeft.setPower(0.01);
////        LiftRight.setPower(0.01);
////
////        MyOpmode.telemetry.addData("Final Lift Position", currentLiftPosition);
////        MyOpmode.telemetry.update();
////    }
////
////    /// returns number of ticks for inch value
////    private int ConvertInchesToTicks(double inches) {
////        // Parameters for encoder wheel.
////        int ticksPerRotation = 2000;
////        double encoderWheelDiameter = 1.82;
////        double encoderWheelRadius = encoderWheelDiameter / 2;
////
////        // Math to figure out ticks per inch.
////        double circumference = 2 * encoderWheelRadius * Math.PI;
////        double ticksPerInch = circumference / ticksPerRotation;
////
////        // Take the lowest whole number.
////        return (int)(Math.floor(inches / ticksPerInch));
////    }
////    public void MoveForwardBackwards(double moveInches, double timeout){
////        int moveTicks = ConvertInchesToTicks(moveInches);
////
////        YEncoderWheel.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
////        YEncoderWheel.setTargetPosition(moveTicks);
////
////        if (moveInches > 0){
////            FrontLeft.setPower(0.5);
////            FrontRight.setPower(0.5);
////            BackLeft.setPower(0.5);
////            BackRight.setPower(0.5);
////        }
////        else if (moveInches < 0) {
////            FrontLeft.setPower(-0.5);
////            FrontRight.setPower(-0.5);
////            BackLeft.setPower(-0.5);
////            BackRight.setPower(-0.5);
////        }
////        else {
////            return;
////        }
////
////        double startTime = MyOpmode.getRuntime();
////        double runTime;
////
////        while (true){
////
////            runTime = MyOpmode.getRuntime() - startTime;
////            if (runTime == timeout){
////                break;
////            }
////
////            MyOpmode.telemetry.addData("Distance travelled", YEncoderWheel.getCurrentPosition());
////            MyOpmode.telemetry.update();
////        }
////
////        FrontLeft.setPower(0.0);
////        FrontRight.setPower(0.0);
////        BackLeft.setPower(0.0);
////        BackRight.setPower(0.0);
////    }
////}
//
//
////        if (positionChangeAmount == 0) {
////            return;
////        } else if (positionChangeAmount > 0) {
////            FrontLeft.setPower(-0.3);
////            FrontRight.setPower(-0.3);
////            BackLeft.setPower(-0.3);
////            BackRight.setPower(-0.3);
////        } else { //this is backwards
////            FrontLeft.setPower(0.3);
////            FrontRight.setPower(0.3);
////            BackLeft.setPower(0.3);
////            BackRight.setPower(0.3);
////        }
////
////        double startPosition = GetYEncoderPosition();
////        double targetPosition = startPosition + positionChangeAmount;
////        double tolerance = 0.25;
////
////        double startTime = MyOpmode.getRuntime();
////        double travelDistance = startPosition - GetYEncoderPosition();
////        double elapsedTime;
////
////        double lowerBound = targetPosition - tolerance;
////        double upperBound = targetPosition + tolerance;
////
////        while (travelDistance < lowerBound || travelDistance > upperBound) {
////            elapsedTime = MyOpmode.getRuntime() - startTime;
////            startPosition = GetYEncoderPosition();
////            travelDistance = targetPosition - startPosition;
////
////            if (elapsedTime >= maxTimeInSeconds) {
////                MyOpmode.telemetry.addData("Error", "Move time out");
////                break;
////            }
////
////            MyOpmode.telemetry.addData("Elapsed", elapsedTime);
////            MyOpmode.telemetry.addData("CurrDistance", startPosition);
////            MyOpmode.telemetry.addData("TravelDistance", travelDistance);
////            MyOpmode.telemetry.update();
////        }
////
////        FrontLeft.setPower(0.0);
////        FrontRight.setPower(0.0);
////        BackLeft.setPower(0.0);
////        BackRight.setPower(0.0);
////    }
}