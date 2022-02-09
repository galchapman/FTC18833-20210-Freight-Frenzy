package edu.megiddo.lions.tokens;

import edu.megiddo.lions.Tokenizer;

public interface TokenParser {
    default String getName() {
        return getClass().getName();
    }

    double parseDouble(Tokenizer.Token token) throws TokenFormatException;
    boolean parseBoolean(Tokenizer.Token token) throws TokenFormatException;
}
