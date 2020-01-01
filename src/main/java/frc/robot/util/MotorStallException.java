package frc.robot.util;

public class MotorStallException extends IllegalStateException {
	public MotorStallException(String msg) {
		super(msg);
	}
}
