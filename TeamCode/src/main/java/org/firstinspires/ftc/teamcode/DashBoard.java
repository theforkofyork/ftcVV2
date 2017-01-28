package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.AdafruitIMU;

/**
 * Created by Archish on 12/27/16.
 */

public class DashBoard {
    public DashBoard(org.firstinspires.ftc.robotcore.external.Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static DashBoard getDash(){
        return instance;
    }
    private static DashBoard instance;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    public void create(String string) {
        telemetry.addLine(string);
    }
    public void create(String string, Object data) {
        telemetry.addData(string, data);
    }


    public void createSticky(String string){
        telemetry.log().add(string);
        update();
    }
    public void createSticky(String string, Object data){
        telemetry.log().add(string, data);
        update();
    }
    public void setNewFirst() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
    }
    public void setNewLast() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
    }
    public void update () {
        telemetry.update();
    }


}