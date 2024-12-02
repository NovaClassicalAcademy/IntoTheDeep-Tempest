
package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ParkAuto", group = "Autonomous")


public class AutonomousRun extends LinearOpMode {
    private Robot myRobot;

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void runOpMode() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing robot");
        myRobot = new Robot(hardwareMap);
        telemetry.addData("Status", "Robot initialized");

        waitForStart();

        //first element deposit after start
        MoveToDumpTower();
        ExtendLift();
        DumpBucket();
        RetractLift();

        //second element pick up and dump
        MoveToElement();
        PickUpElement();
        MoveElementToDumpTower();
        ExtendLift();
        DumpBucket();
        RetractLift();

        //park in observation zone
        MoveToZone();
    }

    private void MoveToDumpTower(){
        myRobot.Move();
        myRobot.Strafe();
        myRobot.Turn();
    }

    private void ExtendLift(){
        myRobot.LiftLift();
    }

    private void DumpBucket(){
        myRobot.Dump();
    }
    private void RetractLift(){
        myRobot.LowerLift();
    }
    private void MoveToElement(){
        myRobot.Move();
        myRobot.Strafe();
        myRobot.Turn();
    }
    private void PickUpElement(){
        myRobot.LowerHinge();
        myRobot.Claw();
        myRobot.LiftHinge();
    }
    private void MoveElementToDumpTower(){

    }
    private void MoveToZone(){

    }


}