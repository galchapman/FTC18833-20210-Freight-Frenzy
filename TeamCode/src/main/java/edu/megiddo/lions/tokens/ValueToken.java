package edu.megiddo.lions.tokens;

import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.megiddo.lions.DistanceUnit;
import edu.megiddo.lions.Tokenizer;

public class ValueToken implements TokenParser {
    private static final Pattern pattern = Pattern.compile("(-?\\d+\\.?\\d*)(m|cm|mm|in|ft|deg|rad)?");
    private final DistanceUnit TargetUnit;

    public ValueToken(DistanceUnit targetUnit) {
        TargetUnit = targetUnit;
    }

    @Override
    public double parseDouble(Tokenizer.Token token) throws TokenFormatException {
        Matcher match = pattern.matcher(token.value);
        if (match.find()) {
            double value = Double.parseDouble(Objects.requireNonNull(match.group(1)));
            String unit = match.group(2);
            if (unit == null) {
                return value;
            } else if (unit.equals("m")) {
                return DistanceUnit.Meter.convertTo(TargetUnit, value);
            } else if (unit.equals("cm")) {
                return DistanceUnit.CentiMeters.convertTo(TargetUnit, value);
            } else if (unit.equals("mm")) {
                return DistanceUnit.MilliMeters.convertTo(TargetUnit, value);
            } else if (unit.equals("in")) {
                return DistanceUnit.Inch.convertTo(TargetUnit, value);
            } else if (unit.equals("ft")) {
                return DistanceUnit.Feet.convertTo(TargetUnit, value);
            } else if (unit.equals("deg")) {
                return Math.toRadians(value);
            } else if (unit.equals("rad")) {
                return value;
            } else {
                throw new TokenFormatException("Invalid distance unit `" + unit + '`', token);
            }
        } else {
            throw new TokenFormatException("token `" + token.value + "` is invalid format", token);
        }
    }

    @Override
    public boolean parseBoolean(Tokenizer.Token token) throws TokenFormatException {
        throw new TokenFormatException("Value can't be boolean", token);
    }
}
