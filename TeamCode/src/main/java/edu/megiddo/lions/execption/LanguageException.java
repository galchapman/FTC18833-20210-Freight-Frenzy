package edu.megiddo.lions.execption;

public class LanguageException extends RuntimeException {
    public final int line;
    public final int character;

    public LanguageException(String type, String error, int line, int character) {
        this(':' + line + ':' + character + " " + type + ": " + error, line, character);
    }

    private LanguageException(String error, int line, int character) {
        super(error);
        this.line = line;
        this.character = character;
    }

    public LanguageException setFile(String filename) {
        return new LanguageException(filename + getMessage(), line, character);
    }
}
