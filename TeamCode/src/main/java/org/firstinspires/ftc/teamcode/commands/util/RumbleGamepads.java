package org.firstinspires.ftc.teamcode.commands.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RumbleGamepads extends CommandBase {
    private final Gamepad[] m_gamepads;
    private final Gamepad.RumbleEffect m_effect;
    private final static Pattern pattern = Pattern.compile("(\\d.?\\d+l)?(\\d.?\\d+s)?(\\d.?\\d+t)");

    public RumbleGamepads(Gamepad.RumbleEffect effect, Gamepad... gamepads) {
        this.m_gamepads = gamepads;
        this.m_effect = effect;
    }

    public RumbleGamepads(String effect, Gamepad... gamepads) {
        m_gamepads = gamepads;
        var parts = effect.split("\\|");
        var builder = new Gamepad.RumbleEffect.Builder();
        for (var part : parts) {
            Matcher matcher = pattern.matcher(part);
            if (matcher.find()) {
                var l = matcher.group(1);
                var s = matcher.group(2);
                var d = matcher.group(3);
                builder.addStep(
                        l == null ? 0 : Double.parseDouble(l.substring(0, -1)),
                        s == null ? 0 : Double.parseDouble(s.substring(0, -1)),
                        d == null ? 0 : Integer.parseInt(d)
                );
            }
        }
        m_effect = builder.build();
    }

    @Override
    public void initialize() {
        for (Gamepad gamepad : m_gamepads)
            gamepad.runRumbleEffect(m_effect);
    }
}
