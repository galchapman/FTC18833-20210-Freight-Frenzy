package edu.megiddo.lions.execption;

public class SyntaxException extends LanguageException {
    public SyntaxException(String error, int line, int character) {
        super("SyntaxException", error, line, character);
    }
}
