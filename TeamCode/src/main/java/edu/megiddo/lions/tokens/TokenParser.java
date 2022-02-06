package edu.megiddo.lions.tokens;

public interface TokenParser {
    default String getName() {
        return getClass().getName();
    }

    double parseDouble(String text) throws TokenException;
    boolean parseBoolean(String text) throws TokenException;
}
