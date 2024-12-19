
package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ParkAutoTestRun", group = "Autonomous")


public class AutonomousRun extends LinearOpMode {

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void runOpMode() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing robot");
        Robot myRobot = new Robot(this);
        telemetry.addData("Status", "Robot initialized");

        waitForStart();
        telemetry.addData("Lift Position", myRobot.GetLiftCurrentPosition());
        telemetry.update();
        sleep(2000);

        myRobot.MoveLift(8.0); //TODO: is this halfway? find full and low
        sleep(2000);
        myRobot.Dump();
        sleep(2000);
        myRobot.MoveLift(0.5);
        sleep(2000);


//        //positive is going forward, negative is going back. might need to change powers. right is lighter than left side.
//        myRobot.Move(-20);

//        //first element deposit after start
//
//
//        //second element pick up and dump
//
//
//        //park in observation zone

    }
}