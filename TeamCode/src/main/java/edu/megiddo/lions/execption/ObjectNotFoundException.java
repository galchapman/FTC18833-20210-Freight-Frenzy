package edu.megiddo.lions.execption;

import edu.megiddo.lions.Tokenizer;

public class ObjectNotFoundException extends LanguageException {
    public ObjectNotFoundException(Tokenizer.Token token) {
        super("ObjectNotFoundException", token.value + " of type " + token.type + " Not found.", token.line, token.character);
    }
}
