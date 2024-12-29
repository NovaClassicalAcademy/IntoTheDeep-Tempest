
package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ParkAutoTestRun", group = "Autonomous")

public class AutonomousRun extends LinearOpMode {
    private void TestLandD (Robot myRobot){
        myRobot.MoveLift(2900, 10);
        sleep(2000);
        myRobot.Dump();
        sleep(2000);
        myRobot.RetractDump();
        sleep(2000);
        myRobot.MoveLift(10, 10);
        sleep(2000);
        //positive is going forward, negative is going back. might need to change powers. right is lighter than left side.
        myRobot.Move(-20);
    }
    private void HighBucket (Robot myRobot){
        ///try this out. i think this is the plan that will score in high bucket.

        //step 1: given high bucket
        myRobot.Move(-5);
        myRobot.Turn(-45);
        myRobot.Move(-20);
        myRobot.Turn(-90);
        myRobot.MoveLift(2900, 10);
        myRobot.Move(-10);
        myRobot.Dump();
        sleep(1000);
        myRobot.RetractDump();
        myRobot.Move(10);
        myRobot.MoveLift(10,10);

        //step 2: furthest element from the wall on the ground score
        myRobot.Turn(-90);
        myRobot.Move(3);
        myRobot.Turn(45);
        myRobot.LowerHinge();
        myRobot.Move(3);
        myRobot.CloseClaw();
        myRobot.LiftHinge();
        myRobot.Move(-3);
        myRobot.Turn(-45);
        myRobot.Move(-3);
        myRobot.Turn(90);
        myRobot.OpenClaw();
        sleep(1000);
        myRobot.MoveLift(2900, 10);
        myRobot.Move(-10);
        myRobot.Dump();
        sleep(1000);
        myRobot.RetractDump();
        myRobot.Move(10);
        myRobot.MoveLift(10,10);

        //step 3: middle element on the ground score
        myRobot.Turn(-90);
        myRobot.Move(5);  //TODO: play around with this number
        myRobot.Turn(45);
        myRobot.LowerHinge();
        myRobot.Move(5);
        myRobot.CloseClaw();
        myRobot.LiftHinge();
        myRobot.Move(-3);
        myRobot.Turn(-45);
        myRobot.Move(-3);
        myRobot.Turn(90);
        myRobot.OpenClaw();
        sleep(1000);
        myRobot.MoveLift(2900, 10);
        myRobot.Move(-10);
        myRobot.Dump();
        sleep(1000);
        myRobot.RetractDump();
        myRobot.Move(10);
        myRobot.MoveLift(10,10);

    }
    private void Specimen(Robot myRobot){

        ///this is plan for specimen
        myRobot.Turn(45);
        myRobot.Move(30);
        myRobot.Turn(90);
        myRobot.Move(10);
        myRobot.Turn(90);
        myRobot.Move(30);
        myRobot.Turn(90);

    }

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void runOpMode() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing robot");
        Robot myRobot = new Robot(this);
        telemetry.addData("Status", "Robot initialized");

        waitForStart();

    }
}