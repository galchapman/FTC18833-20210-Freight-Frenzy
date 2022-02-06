package edu.megiddo.lions.tokens;

public class TokenUnimplementedException extends TokenException {
    public TokenUnimplementedException(String name) {
        super(String.format("UnimplementedException: Token `%s` is not implemented", name));
    }
}
