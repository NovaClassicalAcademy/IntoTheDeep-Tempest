
package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ParkAutoTestRun", group = "Autonomous")


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
//
//        //TODO: can't stop by itself. Maybe get new encoder to read left to right. might need to be timed.
//        myRobot.Strafe(15);
//        sleep(1000);
//        myRobot.Strafe(-15);


//
//        //positive is going forward, negative is going back but this works. might need to change powers. right is lighter than left side.
//        myRobot.Move(-20);

//        //first element deposit after start
//        MoveToDumpTower();
//        ExtendLift();
//        DumpBucket();
//        RetractLift();
//
//        //second element pick up and dump
//        MoveToElement();
//        PickUpElement();
//        MoveElementToDumpTower();
//        ExtendLift();
//        DumpBucket();
//        RetractLift();
//
//        //park in observation zone
//        MoveToZone();
    }

//    private void MoveToDumpTower(){
//        myRobot.Move(3.5);
//        myRobot.Strafe(2);
//        myRobot.Turn(5);
//    }
//
//    private void ExtendLift(){
//        myRobot.LiftLift();
//    }
//
//    private void DumpBucket(){
//        myRobot.Dump();
//    }
//    private void RetractLift(){
//        myRobot.LowerLift();
//    }
//    private void MoveToElement(){
//        myRobot.Move();
//        myRobot.Strafe();
//        myRobot.Turn();
//    }
//    private void PickUpElement(){
//        myRobot.LowerHinge();
//        myRobot.Claw();
//        myRobot.LiftHinge();
//    }
//    private void MoveElementToDumpTower(){
//
//    }
//    private void MoveToZone(){
//
//    }
//
//
}