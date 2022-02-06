package edu.megiddo.lions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import edu.megiddo.lions.commands.Command;

public class Interpreter<T> {
    private final Map<String, Command<T>> commands;
    public final Environment env;

    public Interpreter() {
        this(DistanceUnit.Meter);
    }

    public Interpreter(DistanceUnit distanceUnit) {
        env = new Environment(distanceUnit);
        commands = new HashMap<>();
    }

    public List<T> interpret(String text) throws SyntaxException {
        Queue<Tokenizer.Token> tokens = Tokenizer.getTokens(text);
        List<T> results = new LinkedList<>();

        while (!tokens.isEmpty()) {
            var token = tokens.remove();
            switch (token.type) {
                case Command:
                    results.add(runCommand(token.value, tokens));
                    break;
                case If:
                    Tokenizer.Token condition_token = tokens.remove();
                    if (condition_token.type != TokenType.Variable) {
                        throw new SyntaxException("Condition must be a boolean variable");
                    }
                    // main if
                    boolean condition = env.getBooleanVariable(condition_token.value);
                    nextLine(tokens);
                    parseLogicBlock(tokens, condition, results);
                    // elif
                    while (!tokens.isEmpty() && tokens.peek().type == TokenType.Elif) {
                        tokens.remove();
                        if (condition) {
                            nextLine(tokens);
                            parseLogicBlock(tokens, false, results);
                        } else {
                            condition_token = tokens.remove();
                            nextLine(tokens);
                            if (condition_token.type != TokenType.Variable) {
                                throw new SyntaxException("Condition must be a boolean variable");
                            }
                            condition = env.getBooleanVariable(condition_token.value);
                            parseLogicBlock(tokens, condition, results);
                        }
                    }
                    // else
                    if (!tokens.isEmpty() && tokens.peek().type == TokenType.Else) {
                        tokens.remove();
                        nextLine(tokens);
                        // Runs only if all ifs and elifs failed
                        parseLogicBlock(tokens, !condition, results);
                    }
                    break;
                case NewLine:
                case Indent:
                case Comment:
                    nextLine(tokens);
                    break;
                default:
                    throw new SyntaxException("Invalid token");
            }
        }
        return results;
    }

    public void registerCommand(String name, Command<T> command) {
        commands.put(name, command);
    }

    private T runCommand(String name, Queue<Tokenizer.Token> tokens) throws SyntaxException {
        try {
            return commands.get(name).run(env, getCommandArgs(tokens));
        } catch (Exception exception) {
            System.out.println("Error: " + exception);
            return null;
        }
    }

    private List<Tokenizer.Token> getCommandArgs(Queue<Tokenizer.Token> tokens) throws SyntaxException {
        List<Tokenizer.Token> args = new ArrayList<>();

        while (!tokens.isEmpty()) {
            Tokenizer.Token arg = tokens.remove();
            if (arg.type == TokenType.NewLine
                    || arg.type == TokenType.Comment
                    || arg.type == TokenType.Indent) {
                break;
            } else if (arg.type == TokenType.Value || arg.type == TokenType.Variable) {
                args.add(arg);
            } else {
                throw new SyntaxException("unexpected token type `" + arg.value + '`');
            }
        }
        return args;
    }

    private void parseLogicBlock(Queue<Tokenizer.Token> tokens, boolean execute_block, List<T> results) throws SyntaxException {
        while (!tokens.isEmpty() && (tokens.peek().type == TokenType.Indent
                || tokens.peek().type == TokenType.NewLine
                || tokens.peek().type == TokenType.Comment)) {
            Tokenizer.Token pre_token = tokens.remove();
            if (pre_token.type == TokenType.NewLine || pre_token.type == TokenType.Comment)
                continue;

            Tokenizer.Token token = tokens.remove();
            if (token.type == TokenType.Command) {
                if (execute_block) {
                    results.add(runCommand(token.value, tokens));
                } else {
                    nextLine(tokens); // skip command
                }
            } else if (token.type != TokenType.NewLine && token.type != TokenType.Comment) {
                throw new SyntaxException("Unexpected token: " + token);
            }
        }
    }

    private void nextLine(Queue<Tokenizer.Token> tokens) {
        do {
            while (!tokens.isEmpty() && tokens.peek().type != TokenType.NewLine) {
                tokens.remove();
            }
        } while(!tokens.isEmpty() && tokens.peek().type == TokenType.Comment);
        if (!tokens.isEmpty()) {
            tokens.remove();
        }
    }
}
