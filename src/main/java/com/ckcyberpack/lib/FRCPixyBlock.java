package com.ckcyberpack.lib;

public class FRCPixyBlock {

    private int x;
    private int y;
    private int width;
    private int height;
    private int age;
    private int idxTracking;

    public FRCPixyBlock(int x, int y, int width, int height, int age, int idxTracking) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.age = age;
        this.idxTracking = idxTracking;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getAge() {
        return age;
    }

    public int getIdxTracking() {
        return idxTracking;
    }
}
