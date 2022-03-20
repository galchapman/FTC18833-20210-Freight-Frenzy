package org.commandftc;

import com.qualcomm.robotcore.hardware.Gamepad;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class represents an FTC gamepad that supports the use of lambda expressions to assign commands.
 * This class defined every button on the controller, so we can use it with WPI commands.
 * Note: This code was automatically generated using python.
 */
public class Gp {
    private final Gamepad gp;
	public final Button a;
    public final Button b;
    public final Button x;
    public final Button y;

    public final Button dpad_down;
    public final Button dpad_up;
    public final Button dpad_left;
    public final Button dpad_right;

    public final Button left_bumper;
    public final Button right_bumper;

    public final Button left_stick_button;
    public final Button right_stick_button;

    public final Button back;
    public final Button circle;
    public final Button guide;
    public final Button options;

	public Gp(Gamepad gamepad) {
        gp = gamepad;

		a = new Button(() -> gp.a);
		b = new Button(() -> gp.b);
		x = new Button(() -> gp.x);
		y = new Button(() -> gp.y);

		dpad_down = new Button(() -> gp.dpad_down);
		dpad_up = new Button(() -> gp.dpad_up);
		dpad_left = new Button(() -> gp.dpad_left);
		dpad_right = new Button(() -> gp.dpad_right);

		left_bumper = new Button(() -> gp.left_bumper);
		right_bumper = new Button(() -> gp.right_bumper);

		left_stick_button =  new Button(() -> gp.left_stick_button);
		right_stick_button =  new Button(() -> gp.right_stick_button);

        back = new Button(() -> gp.back);
        circle = new Button(() -> gp.circle);
        guide = new Button(() -> gp.guide);
        options = new Button(() -> gp.options);
	}
}