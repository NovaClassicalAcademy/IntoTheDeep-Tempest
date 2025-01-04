package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareRobot {
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor BackLeftDrive = null;
    public DcMotor BackRightDrive = null;
    public DcMotor YEncoderWheel = null;
    public DcMotor LeftLiftMotor = null;
    public DcMotor RightLiftMotor = null;
    public Servo DumpBucketServo = null;
    public Servo LeftClawServo = null;
    public Servo RightClawServo = null;
    public Servo GripperServo = null;
    public Servo LeftHingeServo = null;
    public Servo RightHingeServo = null;
    public IMU TurnGyro = null;

    public HardwareRobot() {

    }

    public void Init(HardwareMap hardwareMap){

        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        YEncoderWheel = hardwareMap.get(DcMotor.class, "EncoderWheel");
        YEncoderWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        YEncoderWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftLiftMotor = hardwareMap.get(DcMotor.class, "LiftLeft");
        RightLiftMotor = hardwareMap.get(DcMotor.class, "LiftRight");

        LeftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DumpBucketServo = hardwareMap.get(Servo.class, "ServoDump");
        LeftClawServo = hardwareMap.get(Servo.class, "ServoClawLeft");
        RightClawServo = hardwareMap.get(Servo.class, "ServoClawRight");

        GripperServo = hardwareMap.get(Servo.class, "ServoGrip");

        LeftHingeServo = hardwareMap.get(Servo.class, "HingeLeft");
        RightHingeServo = hardwareMap.get(Servo.class, "HingeRight");

        TurnGyro = hardwareMap.get(IMU.class, "GyroSensor");
    }
}
