package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.concurrent.Callable;



public class Tuple<T> {
    public T left;
    public T right;
    public Tuple(T left, T right) {
        this.left = left;
        this.right = right;
    }

    public void apply(TupleApply<T> func) {
        func.apply(this.left);
        func.apply(this.right);
    }

    @Override
    public String toString() {
        return left.toString() + " " + right.toString();
    }
}
