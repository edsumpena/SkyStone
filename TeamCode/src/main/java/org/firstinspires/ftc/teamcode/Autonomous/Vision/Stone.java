package org.firstinspires.ftc.teamcode.Autonomous.Vision;

public class Stone {
    private float leftX;
    private float topX;
    private float bottomX;
    private float rightX;
    private float height;
    private float width;
    private String label;

    public Stone(String label, float leftX, float topX, float height, float width){
        this.label = label;
        this.leftX = leftX;
        this.topX = topX;
        this.height = height;
        this.width = width;
        rightX = leftX + width;
        bottomX = topX + height;
    }

    public float getLeft(){
        return leftX;
    }

    public float getRight(){
        return rightX;
    }

    public float getTop(){
        return topX;
    }

    public float getBottom(){
        return bottomX;
    }

    public float getHeight(){
        return height;
    }

    public float getWidth(){
        return width;
    }

    public String getLabel(){
        return label;
    }

    public float[] getCenter(){
        return new float[]{Math.round(leftX + width / 2d), Math.round(topX + height / 2d)};
    }
}
