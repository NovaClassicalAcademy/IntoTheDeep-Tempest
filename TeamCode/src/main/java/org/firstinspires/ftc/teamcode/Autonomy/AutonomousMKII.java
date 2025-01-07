package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Rei Auto Run MKII", group = "Rei")
public class AutonomousMKII extends LinearOpMode {
    HardwareRobot _robot = new HardwareRobot();

    final double _driveSpeed = 0.1;
    final double _turnSpeed = 0.05;
    final double _liftSpeed = 0.9;
    final int _sleepTime = 100;

    @Override
    public void runOpMode() {
        _robot.Init(hardwareMap);

        telemetry.addData("Initialization", "Started");
//        InitializeLiftPosition();
//        InitializeHingeAndClawsPosition();
//        InitializeGripper();

        telemetry.addData("Initialization", "Complete");
        telemetry.addData("Front Left Drive", _robot.FrontLeftDrive.getCurrentPosition());
        telemetry.addData("Front Right Drive", _robot.FrontRightDrive.getCurrentPosition());
        telemetry.addData("Back Left Drive", _robot.BackLeftDrive.getCurrentPosition());
        telemetry.addData("Back Right Drive", _robot.BackRightDrive.getCurrentPosition());
        telemetry.addData("Lift Position", _robot.LeftLiftMotor.getCurrentPosition());

        telemetry.update();

        waitForStart();
        Test2();
    }

    private void HighBucketTest(){
        MoveForwardBackWards(-2, 2);
        SetLiftPosition(2900, 7);
        Dump();
        sleep(2000);
        RetractDump();
        sleep(2000);
        MoveForwardBackWards(2, 2);
        SetLiftPosition(0, 7);
        Turn(80, 7);
        MoveForwardBackWards(1, 1);
        sleep(100);
        MoveForwardBackWards(8, 5);
//        Turn(-5, 3);
        LowerHinge();
        sleep(2000);
        CloseClaw();
        sleep(2000);
        LiftHinge();
        sleep(2000);
        OpenClaw();
        sleep(2000);
    }

    private void Test2(){
        MoveForwardBackWards(-2, 2);
        SetLiftPosition(2900, 7);
        Dump();
        RetractDump();
        MoveForwardBackWards(2, 2);
        SetLiftPosition(0, 7);
        Turn(80, 7);
        MoveForwardBackWards(1, 1);
        MoveForwardBackWards(8, 5);
        LowerHinge();
        CloseClaw();
        LiftHinge();
        OpenClaw();
    }
    private void MoveForwardBackWards(double moveInches, double timeOut) {
        if (!opModeIsActive()) {
            return;
        }

        _robot.YEncoderWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _robot.YEncoderWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int moveTicks = ConvertInchesToTicks(moveInches);
        double currentPosition = _robot.YEncoderWheel.getCurrentPosition();
        double targetPosition = currentPosition + moveTicks;

        double startTime = getRuntime();
        double runTime;

        if (moveInches < 0) {
           DriveForward();

            while (currentPosition > targetPosition) {

                runTime = getRuntime() - startTime;
                currentPosition = _robot.YEncoderWheel.getCurrentPosition();

                if (runTime > timeOut) { break; }

                telemetry.addData("Time Out Time", timeOut);
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Distance travelled", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Move Ticks", moveTicks);
                telemetry.update();
            }
        } else if (moveInches > 0) {
            DriveBackward();

            while (currentPosition < targetPosition) {

                runTime = getRuntime() - startTime;
                currentPosition = _robot.YEncoderWheel.getCurrentPosition();

                if (runTime > timeOut) { break; }

                telemetry.addData("Time Out Time", timeOut);
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Distance travelled", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Move Ticks", moveTicks);
                telemetry.update();
            }
        }

        DriveOff();
    }

    private void SetLiftPosition(double newPosition, double timeOut) {
        if (!opModeIsActive()) {
            return;
        }

        double currPosition = _robot.RightLiftMotor.getCurrentPosition() * -1;
        double positionDifference = 2900 - newPosition;
        double targetPosition = newPosition - positionDifference;


        // Set min/max limits for the lift.
        if (targetPosition > 2900) {
            targetPosition = 2900;
        }

        if (targetPosition < 0) {
            targetPosition = 0;
        }

        double startTime = getRuntime();
        double runTime;

        if (currPosition > targetPosition) {
            _robot.LeftLiftMotor.setPower(_liftSpeed);
            _robot.RightLiftMotor.setPower(_liftSpeed);

            while (currPosition > targetPosition) {
                currPosition = _robot.RightLiftMotor.getCurrentPosition() * -1;
                runTime = getRuntime() - startTime;

                if (runTime > timeOut) {
                    break;
                }

                telemetry.addData("Lift Mode", "Retracting");
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Time Out", timeOut);
                telemetry.addData("Current Position", currPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
            }
        } else if (currPosition < targetPosition) {
            _robot.LeftLiftMotor.setPower(-_liftSpeed);
            _robot.RightLiftMotor.setPower(-_liftSpeed);

            while (currPosition < targetPosition) {
                currPosition = _robot.RightLiftMotor.getCurrentPosition() * -1;
                runTime = getRuntime() - startTime;

                if (runTime > timeOut) {
                    break;
                }

                telemetry.addData("Lift Mode", "Extending");
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Time Out", timeOut);
                telemetry.addData("Current Position", currPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
            }
        }

        if (currPosition > 1500) {
            // If lift is extended more than halfway,
            // give the lift a tiny amount of power
            // to help hold the position.
            _robot.LeftLiftMotor.setPower(0.01);
            _robot.RightLiftMotor.setPower(0.01);
        } else {
            _robot.LeftLiftMotor.setPower(0.0);
            _robot.RightLiftMotor.setPower(0.0);
        }
    }

    private void Turn(double targetAngle, double timeout) {
        if (!opModeIsActive()) {
            return;
        }

        _robot.TurnGyro.resetYaw();

        double currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
        double degreeTolerance = 1;
        double degreeDifference = targetAngle - currAngle;
        double startTime = getRuntime();
        double runTime;
        targetAngle = targetAngle * -1;

        while (degreeDifference > degreeTolerance) {
            if (currAngle > targetAngle) {
                DriveTurnLeft();
            } else {
                DriveTurnRight();
            }

            runTime = getRuntime() - startTime;
            degreeDifference = targetAngle - currAngle;

            telemetry.addData("Angle", currAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Degree Difference", degreeDifference);
            telemetry.update();

            if (runTime > timeout){
                break;
            }
        }

        DriveOff();

        sleep(_sleepTime);
    }

    private void Dump() {
        _robot.DumpBucketServo.setPosition(0.25);
        sleep(_sleepTime);
    }

    private void RetractDump() {
        _robot.DumpBucketServo.setPosition(1);
        sleep(_sleepTime);
    }

    private void OpenClaw() {
        _robot.LeftClawServo.setPosition(0.0);
        _robot.RightClawServo.setPosition(1.0);
        sleep(_sleepTime);
    }

    private void CloseClaw() {
        _robot.LeftClawServo.setPosition(0.44);
        _robot.RightClawServo.setPosition(0.56);
        sleep(_sleepTime);
    }

    private void OpenGripper() {
        _robot.GripperServo.setPosition(0.6);
        sleep(_sleepTime);
    }

    private void CloseGripper() {
        _robot.GripperServo.setPosition(0.8);
        sleep(_sleepTime);
    }

    private void LiftHinge() {
        _robot.LeftHingeServo.setPosition(1.0);
        _robot.RightHingeServo.setPosition(0.25);
        sleep(_sleepTime);
    }

    private void LowerHinge() {
        _robot.LeftHingeServo.setPosition(0.675);
        _robot.RightHingeServo.setPosition(0.575);
        sleep(_sleepTime);
    }

    private  int ConvertInchesToTicks(double inches) {
        int encoderTicksPerRotation = 2000;
        double encoderDiameter = 1.83 / 2.54; //converting cm to inches
        double circumference = encoderDiameter * Math.PI;
        double inchesPerTick = circumference / encoderTicksPerRotation;

        return (int) Math.floor(inches / inchesPerTick);
    }

    private void InitializeLiftPosition() {
        _robot.LeftLiftMotor.setPower(0.05);
        _robot.RightLiftMotor.setPower(0.05);

        sleep(1000);

        _robot.LeftLiftMotor.setPower(0);
        _robot.RightLiftMotor.setPower(0);

        sleep(1000);

        _robot.LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _robot.RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _robot.LeftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _robot.RightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void DriveForward(){
        _robot.FrontLeftDrive.setPower(-_driveSpeed);
        _robot.FrontRightDrive.setPower(-_driveSpeed);
        _robot.BackLeftDrive.setPower(-_driveSpeed);
        _robot.BackRightDrive.setPower(-_driveSpeed);
    }

    private void DriveBackward(){
        _robot.FrontLeftDrive.setPower(_driveSpeed);
        _robot.FrontRightDrive.setPower(_driveSpeed);
        _robot.BackLeftDrive.setPower(_driveSpeed);
        _robot.BackRightDrive.setPower(_driveSpeed);
    }

    private void DriveTurnLeft(){
        _robot.FrontLeftDrive.setPower(-_turnSpeed);
        _robot.FrontRightDrive.setPower(_turnSpeed);
        _robot.BackLeftDrive.setPower(-_turnSpeed);
        _robot.BackRightDrive.setPower(_turnSpeed);
    }

    private void DriveTurnRight(){
        _robot.FrontLeftDrive.setPower(_turnSpeed);
        _robot.FrontRightDrive.setPower(-_turnSpeed);
        _robot.BackLeftDrive.setPower(_turnSpeed);
        _robot.BackRightDrive.setPower(-_turnSpeed);
    }

    private void DriveOff(){
        _robot.FrontLeftDrive.setPower(0.0);
        _robot.FrontRightDrive.setPower(0.0);
        _robot.BackLeftDrive.setPower(0.0);
        _robot.BackRightDrive.setPower(0.0);
    }
    private void InitializeHingeAndClawsPosition(){
        LiftHinge();
        OpenClaw();
        sleep(1000);
        CloseClaw();
        sleep(1000);
        OpenClaw();
        sleep(1000);
        CloseClaw();
        sleep(1000);
    }

    private void InitializeGripper(){
        OpenGripper();
        sleep(1000);
        CloseGripper();
        sleep(1000);
        OpenGripper();
        sleep(1000);
        CloseGripper();
        sleep(1000);
    }
}
