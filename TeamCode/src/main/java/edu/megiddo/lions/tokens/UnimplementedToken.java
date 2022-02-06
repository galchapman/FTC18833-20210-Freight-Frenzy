package edu.megiddo.lions.tokens;

public class UnimplementedToken implements TokenParser {
    private final TokenUnimplementedException error;
    public UnimplementedToken(String name) {
        error = new TokenUnimplementedException(name);
    }

    @Override
    public double parseDouble(String text) throws TokenUnimplementedException {
        throw error;
    }

    @Override
    public boolean parseBoolean(String text) throws TokenUnimplementedException{
        throw error;
    }
}
