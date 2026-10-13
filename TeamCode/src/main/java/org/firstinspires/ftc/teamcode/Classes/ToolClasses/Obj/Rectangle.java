package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj;

public class Rectangle {

    // Coordenadas del rectángulo (esquina superior izquierda)
    private double x;
    private double y;

    // Tamaño
    private double width;
    private double height;

    // Constructor
    public Rectangle(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    // Getters
    public double getX() { return x; }
    public double getY() { return y; }
    public double getWidth() { return width; }
    public double getHeight() { return height; }

    // Setters
    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setWidth(double width) { this.width = width; }
    public void setHeight(double height) { this.height = height; }

    public boolean collidesWith(Rectangle other) {
        return this.x < other.x + other.width  &&
                this.x + this.width > other.x   &&
                this.y < other.y + other.height &&
                this.y + this.height > other.y;
    }

    @Override
    public String toString() {
        return "Rectangle[x=" + x + ", y=" + y +
                ", width=" + width + ", height=" + height + "]";
    }
}
