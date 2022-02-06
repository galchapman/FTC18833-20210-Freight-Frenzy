package edu.megiddo.lions;

public class TokenFormatError extends RuntimeException {
    public TokenFormatError(String error) {
        super(error);
    }
}
