package edu.megiddo.lions.execption;

public class TokenFormatException extends LanguageException {
    public TokenFormatException(String error, int line, int character) {
        super("TokenFormatException", error, line, character);
    }
}
