#include "lexer.hpp"
#include <cctype>
#include <sstream>

namespace planner {

// 位置情報を文字列に変換するヘルパ関数
static std::string locStr(const Location& L) {
    std::ostringstream oss;
    oss << "(line " << L.line << ", col " << L.col << ")";
    return oss.str();
}

// Lexerの本体部分の実装
// コンストラクタ
Lexer::Lexer(const std::string& input) : input_(input) {}

// 現在の行内の位置を1つ進める関数
void Lexer::advance() {
    if (eof()) return;
    if (cur() == '\n') advanceLine();
    else {
        ++i_;
        ++loc_.col;
    }
}

// 現在の行を改行処理する関数 (loc_.lineを1つ進め、loc_.colを1にリセット)
void Lexer::advanceLine() {
    ++i_;
    ++loc_.line;
    loc_.col = 1;
}

// 整数または実数の判定する関数
bool Lexer::isDigit(char c) { 
    return std::isdigit(static_cast<unsigned char>(c)) != 0; // 一度unsigned charに変換してから判定する
}

// PDDLの名前の開始文字であるか判定する関数
bool Lexer::isNameStart(char c) {
    // 許容: 英字、数字、アンダースコア、記号いくつか、ハイフン(独立したものではないもの)、ドット
    unsigned char uc = static_cast<unsigned char>(c);
    if (std::isspace(uc) || c == '(' || c == ')' || c == ';' || c == ':' || c == '?')
        return false;
    return true;
}

// 名前文字の判定を行う関数
bool Lexer::isNameChar(char c) {
    // スペース、括弧、セミコロン以外は名前文字として許容
    unsigned char uc = static_cast<unsigned char>(c);
    if (std::isspace(uc) || c == '(' || c == ')' || c == ';')
        return false;
    return true;
}

// 空白とコメントをスキップする関数
void Lexer::skipWhitespaceAndComments() {
    while (!eof()) {
        // 空白
        if (std::isspace(static_cast<unsigned char>(cur()))) {
            advance();
            continue;
        }
        // セミコロン以降は行末までコメントなのでスキップする
        if (cur() == ';') {
            while (!eof() && cur() != '\n') advance();
            continue;
        }
        break;
    }
}

// トークンを生成するヘルパ関数
Token Lexer::make(TokenType t, std::string lex) {
    return Token{t, std::move(lex), loc_};
}

// エラー処理を行う関数
[[noreturn]] void Lexer::errorHere(const std::string& msg) {
    throw LexerError("Lexer error " + locStr(loc_) + ": " + msg);
}

// 数字をトークンとして読み取る関数
Token Lexer::readNumber() {
    Location start = loc_; // 数字の開始位置
    std::string s; // 数字の格納場所
    bool seen_dot = false; // 小数点を見たかどうか

    while (!eof()) {
        char c = cur();
        if (isDigit(c)) {
            s.push_back(c);
            advance();
        } else if (c == '.' && !seen_dot) {
            seen_dot = true;
            s.push_back(c);
            advance();
        } else {
            break;
        }
    }
    if (s.empty() || s == ".")
        throw LexerError("Invalid number " + locStr(start));
    return Token{TokenType::NUMBER, s, start};
}

// 名前に関するトークンを読み取る関数
Token Lexer::readNameLike() {
    Location start = loc_;

    // KEYWORD（先頭 ':'）
    if (!eof() && cur() == ':') {
        advance(); // consume ':'
        if (eof() || !isNameChar(cur()))
            errorHere("Expected keyword after ':'");
        std::string k;
        while (!eof() && isNameChar(cur())) {
            k.push_back(cur());
            advance();
        }
        return Token{TokenType::KEYWORD, k, start};
    }

    // VARIABLE（先頭 '?'）
    if (!eof() && cur() == '?') {
        advance(); // consume '?'
        if (eof() || !isNameChar(cur()))
            errorHere("Expected variable name after '?'");
        std::string v;
        while (!eof() && isNameChar(cur())) {
            v.push_back(cur());
            advance();
        }
        return Token{TokenType::VARIABLE, v, start};
    }

    // NAME
    if (!eof() && isNameStart(cur())) {
        std::string n;
        while (!eof() && isNameChar(cur())) {
            n.push_back(cur());
            advance();
        }
        // 単独の "-" を NAME と誤判定しないよう補正
        if (n.size() == 1 && n[0] == '-') {
            return Token{TokenType::DASH, "-", start};
        }
        return Token{TokenType::NAME, n, start};
    }

    errorHere("Invalid symbol start");
}

// トークンを先読みする関数
const Token& Lexer::peek() {
    if (lookahead_.has_value())
        return *lookahead_;

    skipWhitespaceAndComments(); // 空白とコメントをスキップ

    if (eof()) { // EOFの場合
        lookahead_ = make(TokenType::EOF_TOKEN);
        return *lookahead_;
    }

    char c = cur(); // 現在の文字を取得

    if (c == '(') {
        Token t = make(TokenType::LPAR, "(");
        advance();
        lookahead_ = std::move(t);
        return *lookahead_;
    }
    if (c == ')') {
        Token t = make(TokenType::RPAR, ")");
        advance();
        lookahead_ = std::move(t);
        return *lookahead_;
    }
    if (c == '-') {
        // 単独の '-' の場合は DASH として扱う
        std::size_t j = i_ + 1;
        if (j >= input_.size() || std::isspace(static_cast<unsigned char>(input_[j])) ||
            input_[j] == '(' || input_[j] == ')' || input_[j] == ';') {
            Token t = make(TokenType::DASH, "-");
            advance();
            lookahead_ = std::move(t);
            return *lookahead_;
        }
        // それ以外は NAME として扱わせる
    }

    if (isDigit(c)) {
        Token t = readNumber();
        lookahead_ = std::move(t);
        return *lookahead_;
    }

    Token t = readNameLike();
    lookahead_ = std::move(t);
    return *lookahead_;
}

Token Lexer::next() {
    const Token& t = peek();
    Token out = t;
    lookahead_.reset();
    return out;
}

// トークンを期待する関数
Token Lexer::expect(TokenType t, const char* what) {
    const Token& p = peek();
    if (p.type != t) {
        std::ostringstream oss;
        oss << "Expected " << (what ? what : "token")
            << " " << locStr(p.loc)
            << " but got ";
        switch (p.type) {
            case TokenType::LPAR:      oss << "'('"; break;
            case TokenType::RPAR:      oss << "')'"; break;
            case TokenType::KEYWORD:   oss << "KEYWORD(" << p.lexeme << ")"; break;
            case TokenType::VARIABLE:  oss << "VARIABLE(" << p.lexeme << ")"; break;
            case TokenType::NAME:      oss << "NAME(" << p.lexeme << ")"; break;
            case TokenType::NUMBER:    oss << "NUMBER(" << p.lexeme << ")"; break;
            case TokenType::DASH:      oss << "'-'"; break;
            case TokenType::EOF_TOKEN: oss << "EOF"; break;
        }
        throw LexerError(oss.str());
    }
    return next();
}

} // namespace planner
