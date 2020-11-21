package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.HzGamepadClassic;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;

/**
 * TeleOpMode for Team Hazmat<BR>
 */
@TeleOp(name = "Test_Magazine", group = "Test")
public class Test_Magazine extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadClassic hzGamepadClassic;

    Magazine hzMagazine;

    @Override
    public void runOpMode() {

        hzMagazine = new Magazine(hardwareMap);

        hzGamepadClassic = new HzGamepadClassic(gamepad1,this);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        hzMagazine.initMagazine(this);

        int keyCount = 0;
        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {

            if (hzGamepadClassic.getButtonXPress()) {
                if (keyCount==0){
                    hzMagazine.turnMagazineBeaconOff();
                    keyCount++;
                } else if (keyCount==1){
                    hzMagazine.turnMagazineBeaconPurple();
                    keyCount++;
                } else if  (keyCount==2){
                    hzMagazine.turnMagazineBeaconTeal();
                    keyCount++;
                } else if (keyCount==3){
                    hzMagazine.turnMagazineBeaconWhite();
                    keyCount = 0;
                }
            }

            if (hzGamepadClassic.getButtonAPress()) {
                hzMagazine.senseMagazineRingStatus();;
            }

            if (hzGamepadClassic.getButtonBPress()){
                hzMagazine.senseMagazinePosition();
            }

            if (hzGamepadClassic.getDpad_upPress()) {
                hzMagazine.moveMagazineToLaunch();
            }

            if (hzGamepadClassic.getDpad_downPress()) {
                hzMagazine.moveMagazineToCollect();
            }

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
        }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("Timing","9:53");

        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        //telemetry.addData("launcherRingPlungerServo.getPosition()", hzLauncher.launcherRingPlungerServo.getPosition());

        telemetry.addData("getDistance(DistanceUnit.CM)",hzMagazine.magazine_distance);

        switch (hzMagazine.getMagazineRingCount()){
            case MAGAZINE_RINGS_0:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_0");
                break;
            }
            case MAGAZINE_RINGS_1:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_1");
                break;
            }
            case MAGAZINE_RINGS_2:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_2");
                break;
            }
            case MAGAZINE_RINGS_3:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_3");
                break;
            }
        }

        switch (hzMagazine.getMagazinePosition()) {
            case MAGAZINE_AT_LAUNCH: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_LAUNCH");
                break;
            }
            case MAGAZINE_AT_COLLECT: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_COLLECT");
                break;
            }
            case MAGAZINE_AT_ERROR: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_ERROR");
                break;
            }
        }

        telemetry.addData("magazineLaunchTouchSensor.getState()", hzMagazine.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("magazineCollectTouchSensor.getState()", hzMagazine.magazineCollectTouchSensor.isPressed());
    }

}

