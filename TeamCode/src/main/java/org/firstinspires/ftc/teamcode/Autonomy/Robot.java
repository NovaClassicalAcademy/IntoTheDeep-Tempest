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
    public final double LiftPower = 0.5;
    private final int StartingYPosition;
    private final int StartingLiftPosition;

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
        Gyro = MyOpmode.hardwareMap.get(IMU.class, "GyroSensor");

        StartingYPosition = YEncoderWheel.getCurrentPosition();
        StartingLiftPosition = LiftRight.getCurrentPosition();
//        StartingXPosition = XEncoderWheel.getCurrentPosition();
    }

    /// returns forward and backward distance
    private double getYDistance() {
        int ticksPerRotation = 2000;
        double Rotations = (double) (YEncoderWheel.getCurrentPosition() - StartingYPosition) / ticksPerRotation;
        double encoderWheelDiameter = 1.82;
        return Rotations * encoderWheelDiameter * Math.PI;
    }

    private double getLiftDistance() {
        int ticksPerRotation = 2000;
        double Rotations = (double) (LiftRight.getCurrentPosition() - StartingLiftPosition) / ticksPerRotation;
        double encoderWheelDiameter = 1.82;
        return Rotations * encoderWheelDiameter * Math.PI* -1;
    }

    public double GetLiftCurrentPosition() {
        return LiftRight.getCurrentPosition();
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

    public void LowerHinge() {
        double currentHingePosition = ServoHingeLeft.getPosition();

        ServoHingeLeft.setPosition(0.675);
        ServoHingeRight.setPosition(0.575);

        while (currentHingePosition != 0.675) {
            currentHingePosition = ServoHingeLeft.getPosition();

            MyOpmode.telemetry.addData("Hinge Rotation", currentHingePosition);
            MyOpmode.telemetry.update();
        }
    }

    public void LiftHinge() {
            double currentHingePosition = ServoHingeLeft.getPosition();

            ServoHingeLeft.setPosition(1.0);
            ServoHingeRight.setPosition(0.25);

            while (currentHingePosition != 1.0) {
                currentHingePosition = ServoHingeLeft.getPosition();

                MyOpmode.telemetry.addData("Hinge Rotation", currentHingePosition);
                MyOpmode.telemetry.update();
            }
    }

    public void OpenClaw() {
        double currentClawPosition = ServoLeft.getPosition();

        ServoLeft.setPosition(0.0);
        ServoRight.setPosition(1.0);

        while (currentClawPosition != 0.0) {
            currentClawPosition = ServoDump.getPosition();

            MyOpmode.telemetry.addData("Claw Rotation", currentClawPosition);
            MyOpmode.telemetry.update();
        }
    }

    public void CloseClaw() {
        double currentClawPosition = ServoLeft.getPosition();

        ServoLeft.setPosition(0.44);
        ServoRight.setPosition(0.56);

        while (currentClawPosition != 0.37) {
            currentClawPosition = ServoDump.getPosition();

            MyOpmode.telemetry.addData("Claw Rotation", currentClawPosition);
            MyOpmode.telemetry.update();
        }
    }

    public void OpenGripper() {
        double currentGripPosition = ServoGrip.getPosition();

        ServoLeft.setPosition(0.15);

        while (currentGripPosition != 0.15) {
            currentGripPosition = ServoGrip.getPosition();

            MyOpmode.telemetry.addData("Grip Rotation", currentGripPosition);
            MyOpmode.telemetry.update();
        }
    }

    public void CloseGripper(){
        double currentGripPosition = ServoGrip.getPosition();

        ServoLeft.setPosition(0.8);

        while (currentGripPosition != 0.3) {
            currentGripPosition = ServoGrip.getPosition();

            MyOpmode.telemetry.addData("Grip Rotation", currentGripPosition);
            MyOpmode.telemetry.update();
        }
    }

    public void Dump() {
        double currentDumpPosition = ServoDump.getPosition();
        double endPosition = 0.25;

        ServoDump.setPosition(endPosition);

        while (currentDumpPosition != endPosition) {
            currentDumpPosition = ServoDump.getPosition();

            MyOpmode.telemetry.addData("Dump Rotation", currentDumpPosition);
            MyOpmode.telemetry.update();
        }
    }

    public void RetractDump() {
        double currentDumpPosition = ServoDump.getPosition();
        double endPosition = 1;

       ServoDump.setPosition(endPosition);

        while (currentDumpPosition != endPosition){
            currentDumpPosition = ServoDump.getPosition();

            MyOpmode.telemetry.addData("Retract Rotation", currentDumpPosition);
            MyOpmode.telemetry.update();
        }
    }

    public void MoveLift(double targetPosition,double maxTimeInSeconds) {
//        double currentLiftPosition = getLiftDistance();
        double currentLiftPosition = LiftLeft.getCurrentPosition()* -1;
        double heightRange = 7;
        double liftPower = -LiftPower;

        if (targetPosition < currentLiftPosition){
            liftPower = LiftPower;
        }

        double startTime = MyOpmode.getRuntime();

        while (Math.abs(currentLiftPosition - targetPosition) > heightRange) {
            double elapsedTime = MyOpmode.getRuntime()-startTime;
            MyOpmode.telemetry.addData("Elapsed", elapsedTime);

            if (elapsedTime >= maxTimeInSeconds){
                MyOpmode.telemetry.addData("Error", "Lift time out");
                break;
            }

            LiftLeft.setPower(liftPower);
            LiftRight.setPower(liftPower);

//            currentLiftPosition = getLiftDistance();
            currentLiftPosition = LiftLeft.getCurrentPosition()* -1;

            MyOpmode.telemetry.addData("Current Lift Position", currentLiftPosition);
            MyOpmode.telemetry.update();
            MyOpmode.telemetry.addData("Target Position", targetPosition);
            MyOpmode.telemetry.update();
        }

        //Once it is in range
        LiftLeft.setPower(0);
        LiftRight.setPower(0);

        MyOpmode.telemetry.addData("Final Lift Position", currentLiftPosition);
        MyOpmode.telemetry.update();
    }
}
