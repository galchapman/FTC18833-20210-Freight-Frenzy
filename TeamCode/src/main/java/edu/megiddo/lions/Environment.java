package edu.megiddo.lions;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.megiddo.lions.tokens.ValueToken;

public class Environment {
    private final Map<String, BooleanSupplier> booleanVariablesRead;
    private final Map<String, Consumer<Boolean>> booleanVariablesWrite;
    private final Map<String, DoubleSupplier> doubleVariablesRead;
    private final Map<String, DoubleConsumer> doubleVariablesWrite;
    private final ValueToken valueTokenParser;

    public Environment(DistanceUnit distanceUnit) {
        booleanVariablesRead = new HashMap<>();
        booleanVariablesWrite = new HashMap<>();
        doubleVariablesRead = new HashMap<>();
        doubleVariablesWrite = new HashMap<>();
        valueTokenParser = new ValueToken(distanceUnit);
    }

    public boolean hasBooleanVarRead(String name) {
        return booleanVariablesRead.containsKey(name);
    }

    public boolean hasBooleanVarWrite(String name) {
        return booleanVariablesWrite.containsKey(name);
    }

    public boolean hasDoubleVarRead(String name) {
        return doubleVariablesRead.containsKey(name);
    }

    public boolean hasDoubleVarWrite(String name) {
        return doubleVariablesWrite.containsKey(name);
    }

    public void addVariable(String name, BooleanSupplier supplier) {
        booleanVariablesRead.put(name, supplier);
    }

    public void addVariable(String name, DoubleSupplier supplier) {
        doubleVariablesRead.put(name, supplier);
    }

    public void addVariable(String name, Consumer<Boolean> function) {
        booleanVariablesWrite.put(name, function);
    }

    public void addVariable(String name, DoubleConsumer function) {
        doubleVariablesWrite.put(name, function);
    }

    public boolean getBooleanVariable(String name) {
        if (name.equalsIgnoreCase("True"))
            return true;
        if (name.equalsIgnoreCase("False"))
            return false;
        return booleanVariablesRead.get(name).getAsBoolean();
    }

    public double getDoubleVariable(String name) {
        return doubleVariablesRead.get(name).getAsDouble();
    }

    public void setBooleanVariable(String name, boolean value) {
        booleanVariablesWrite.get(name).accept(value);
    }

    public void setDoubleVariable(String name, double value) {
        doubleVariablesWrite.get(name).accept(value);
    }

    public double parseValueToken(String token) {
        return valueTokenParser.parseDouble(token);
    }
}
