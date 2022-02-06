package edu.megiddo.lions.tokens;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.megiddo.lions.DistanceUnit;
import edu.megiddo.lions.TokenFormatError;

public class ValueToken implements TokenParser {
    private static final Pattern pattern = Pattern.compile("(\\d+)(m|cm|mm|in|ft)?");
    private final DistanceUnit TargetUnit;

    public ValueToken(DistanceUnit targetUnit) {
        TargetUnit = targetUnit;
    }

    @Override
    public double parseDouble(String text) throws TokenException {
        Matcher match = pattern.matcher(text);
        if (match.find()) {
            double value = Double.parseDouble(match.group(1));
            String unit = match.group(2);
            if (unit == null) {
                return DistanceUnit.Meter.convertTo(TargetUnit, value);
            } else if (unit.equals("m")) {
                return value;
            } else if (unit.equals("cm")) {
                return DistanceUnit.CentiMeters.convertTo(TargetUnit, value);
            } else if (unit.equals("mm")) {
                return DistanceUnit.MilliMeters.convertTo(TargetUnit, value);
            } else if (unit.equals("in")) {
                return DistanceUnit.Inch.convertTo(TargetUnit, value);
            } else if (unit.equals("ft")) {
                return DistanceUnit.Feet.convertTo(TargetUnit, value);
            } else {
                throw new TokenFormatError("Invalid distance unit `" + unit + '`');
            }
        } else {
            throw new TokenFormatError("token `" + text + "` is invalid format");
        }
    }

    @Override
    public boolean parseBoolean(String text) throws TokenException {
        throw new TokenException("Value can't be boolean");
    }
}
