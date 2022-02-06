package edu.megiddo.lions;

import java.nio.charset.StandardCharsets;
import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

import edu.megiddo.lions.tokens.TokenException;

public class Tokenizer {

    public static class Token {
        public final TokenType type;
        public final String value;

        public Token(TokenType type, String value) {
            this.type = type;
            this.value = value;
        }

        @Override
        public String toString() {
            return type + "(`" + value + "`)";
        }
    }

    static public Queue<Token> getTokens(String text) throws TokenException {
        Deque<Token> tokens = new LinkedList<>();

        StringBuilder currentToken = new StringBuilder();
        TokenType token = TokenType.Empty;
        for (var c : text.getBytes(StandardCharsets.UTF_8)) {
            if (c == '\r')
                continue;

            if (((c == ' ' || c == '\t') && token != TokenType.Comment) || c == '\n') {
                if (token == TokenType.Empty && tokens.getLast().type != TokenType.Indent) {
                    tokens.add(new Token(TokenType.Indent, ""));
                } else if (token == TokenType.Logic) {
                    if (currentToken.toString().equalsIgnoreCase("if")) {
                        tokens.add(new Token(TokenType.If, ""));
                    } else if (currentToken.toString().equalsIgnoreCase("elif")) {
                        tokens.add(new Token(TokenType.Elif, ""));
                    } else if (currentToken.toString().equalsIgnoreCase("else")) {
                        tokens.add(new Token(TokenType.Else, ""));
                    } else {
                        throw new TokenException("Invalid token: @" + currentToken);
                    }
                } else {
                    tokens.add(new Token(token, currentToken.toString()));
                }

                if (c == '\n') {
                    tokens.add(new Token(TokenType.NewLine, ""));
                } else if (c == '\t'  && tokens.getLast().type != TokenType.Indent) {
                    tokens.add(new Token(TokenType.Indent, ""));
                }

                token = TokenType.Empty;
                currentToken = new StringBuilder();
                continue;
            } else if (token == TokenType.Empty) {
                switch (c) {
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        token = TokenType.Value;
                        break;
                    case '#':
                        token = TokenType.Comment;
                        continue;
                    case '@':
                        token = TokenType.Logic;
                        continue;
                    default:
                        if (tokens.isEmpty() || tokens.getLast().type == TokenType.NewLine || tokens.getLast().type == TokenType.Indent) {
                            token = TokenType.Command;
                        } else {
                            token = TokenType.Variable;
                        }
                }
            }
            currentToken.append((char)c);
        }

        if (token != TokenType.Empty)
            tokens.add(new Token(token, currentToken.toString()));

        return tokens;
    }
}
