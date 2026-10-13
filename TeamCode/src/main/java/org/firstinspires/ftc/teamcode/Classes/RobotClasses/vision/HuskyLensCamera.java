package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision;

import android.animation.RectEvaluator;
import android.graphics.drawable.shapes.Shape;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVisionColorLocator_Rectangle;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Rectangle;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;



public class HuskyLensCamera {

    private static final double FocalLength = 334.42;
    private static final double TagSize = 0.165;
    private HuskyLens camera;
    private final int READ_PERIOD = 1;
    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);



    public HuskyLensCamera(LinearOpMode linear){
        this.camera = linear.hardwareMap.get(HuskyLens.class, "huskylens");
        rateLimit.expire();

    }
    public void update(){
        rateLimit.reset();
    }
    public HuskyLens.Block[] getBloks(){
        return  camera.blocks();
    }

    public void setAlgorithm(HuskyLens.Algorithm algorithm){
        camera.selectAlgorithm(algorithm);


        rateLimit.expire();
    }

    ///////////////
    public double getGreenX(){
        if(getBloks() == null){
            return  999999;
        }

        HuskyLens.Block targetBlock = null;
        boolean x = false;


        HuskyLens.Block[] bloks= getBloks();

        if(bloks != null){

            for(int i = 0; i < bloks.length; i++){


                    if (bloks[i].id == 2 && bloks[i].height*bloks[i].width > 10) {
                        x = true;
                        targetBlock = bloks[i];
                    } else if (bloks[i].id == 2 && x && bloks[i].height*bloks[i].width > 10) {
                        if (bloks[i].width * bloks[i].height > targetBlock.width * targetBlock.height) {
                            targetBlock = bloks[i];
                        }

                    }


            }
        }
        if(targetBlock == null){
            return 999999;
        }else{
            return targetBlock.x;
        }

    }
    public double getPurpleX(){
        if(getBloks() == null){
            return  999999;
        }

        HuskyLens.Block targetBlock = null;
        boolean x = false;


        HuskyLens.Block[] bloks= getBloks();

        if(bloks != null){

            for(int i = 0; i < bloks.length; i++){


                if (bloks[i].id == 1 && bloks[i].height*bloks[i].width > 10) {
                    x = true;
                    targetBlock = bloks[i];
                } else if (bloks[i].id == 1 && x && bloks[i].height*bloks[i].width > 10) {
                    if (bloks[i].width * bloks[i].height > targetBlock.width * targetBlock.height) {
                        targetBlock = bloks[i];
                    }

                }


            }
        }
        if(targetBlock == null){
            return 999999;
        }else{
            return targetBlock.x;
        }

    }


    public double getPurpleX2(){
        if(getBloks() == null){
            return  999999;
        }

        double returnX = 0;

        HuskyLens.Block[] bloks= getBloks();
        ArrayList<HuskyLens.Block> filtredBlocks = new ArrayList<>();

        if(bloks != null) {
            HuskyLens.Block delta;
            for (int i = 0; i < bloks.length; i++) {
                if (bloks[i].id == 1 || bloks[i].id > 2) {
                    filtredBlocks.add(bloks[i]);
                }
            }
            if (filtredBlocks.size() > 1){
                int counter = 0;
                delta = filtredBlocks.get(0);
                for (int i = 0; i < filtredBlocks.size(); i++) {

                    if(isColiding(filtredBlocks.get(i), delta)) {
                        returnX += filtredBlocks.get(i).x;
                    }else{
                        counter++;
                    }
                    delta = filtredBlocks.get(i);
                }
                returnX /= (filtredBlocks.size()-counter);
            }
        if(returnX == 0){
            returnX =999999;
        }

            return returnX;

        }else{
            return 999999;
        }


    }

    public double getTagX(){
        if(getBloks() == null){
            return  999999;
        }

        HuskyLens.Block targetBlock = null;
        boolean x = false;


        HuskyLens.Block[] bloks= getBloks();

        if(bloks != null){

            for(int i = 0; i < bloks.length; i++){


                if (bloks[i].id == 1 && bloks[i].height*bloks[i].width > 10) {
                    x = true;
                    targetBlock = bloks[i];
                } else if (bloks[i].id == 1 && x && bloks[i].height*bloks[i].width > 10) {
                    if (bloks[i].width * bloks[i].height > targetBlock.width * targetBlock.height) {
                        targetBlock = bloks[i];
                    }

                }


            }
        }
        if(targetBlock == null){
            return 999999;
        }else{
            return targetBlock.x;
        }

    }
    public double getTagDistance(int index){
        if(getBloks() == null){
            return  999999;
        }

        HuskyLens.Block targetBlock = null;
        boolean x = false;


        HuskyLens.Block[] bloks= getBloks();

        if(bloks != null){

            for(int i = 0; i < bloks.length; i++){


                if (bloks[i].id == index && bloks[i].height*bloks[i].width > 10) {
                    x = true;
                    targetBlock = bloks[i];
                } else if (bloks[i].id == index && x && bloks[i].height*bloks[i].width > 10) {
                    if (bloks[i].width * bloks[i].height > targetBlock.width * targetBlock.height) {
                        targetBlock = bloks[i];
                    }

                }


            }
        }
        if(targetBlock == null){
            return 999999;
        }else{
            return (FocalLength * TagSize)/targetBlock.width;
        }

    }
    public double getDistanceXTag(int index){
        double z = getTagDistance(index);

        if(z != 999999) {
            return ((getTagX() - 150) * getTagDistance(index)) / FocalLength;
        }else{
            return 999999;
        }

    }
    public double getDistanceYTag(int index){

        double z = getTagDistance(index);
        if(z == 999999){
            return 999999;
        }

        double x = getDistanceXTag(index);
       return (z*z)-(x*x);
    }



private boolean isColiding(HuskyLens.Block x, HuskyLens.Block y){
    Rectangle r = new Rectangle(x.x, x.y, x.width, x.height);
    Rectangle r2 = new Rectangle(y.x, y.y, y.width, y.height);

    return r.collidesWith(r2);
}

}
