#include "parser.hpp"
#include <unordered_set>
#include <unordered_map>
#include <stdexcept>
#include <sstream>

namespace planner {

// 位置情報を付加するヘルパ関数
static std::string loc(const Token& t){
    std::ostringstream oss;
    oss << " at " << t.loc.line << ":" << t.loc.col;
    return oss.str();
}

// ---文字列ユーティリティ関数---
// 命題を文字列化する関数
std::string Parser::to_string(const Atom& a){
    std::ostringstream oss;
    oss << "(" << a.pred;
    for (auto& s : a.args) oss << " " << s;
    oss << ")";
    return oss.str();
}

// 論理式を文字列化する関数
std::string Parser::to_string(const Formula& f){
    std::ostringstream oss;
    if (f.kind == Formula::ATOM) {
        oss << to_string(f.atom);
    } else if (f.kind == Formula::AND) {
        oss << "(and";
        for (auto& c : f.children) oss << " " << to_string(c);
        oss << ")";
    } else if (f.kind == Formula::INCREASE) {
        oss << "(increase " << to_string(f.inc.lhs) << " " << to_string(f.inc.rhs) << ")";
    } else {
        oss << "(not " << to_string(*f.child) << ")";
    }
    return oss.str();
}

// 関数項を文字列化する関数
std::string Parser::to_string(const FuncTerm& ft){
    std::ostringstream oss;
    oss << "(" << ft.name;
    for (auto& s : ft.args) oss << " " << s;
    oss << ")";
    return oss.str();
}

// 数値式を文字列化する関数
std::string Parser::to_string(const NumExpr& ne){
    if (ne.kind == NumExpr::CONST) { // 定数の場合
        std::ostringstream oss; oss << ne.value; return oss.str();
    }
    if (ne.kind == NumExpr::FUNC) { // 関数の場合
        return to_string(ne.func);
    }
    // 演算子ノード
    char sym = '?';
    switch (ne.kind) {
        case NumExpr::ADD: sym = '+'; break;
        case NumExpr::SUB: sym = '-'; break;
        case NumExpr::MUL: sym = '*'; break;
        case NumExpr::DIV: sym = '/'; break;
        default: break;
    }
    std::ostringstream oss;
    oss << "(" << sym;
    for (auto& a : ne.args) oss << " " << to_string(a);
    oss << ")";
    return oss.str();
}



// ---予想したトークンを取得する関数---
// 名前を取得する関数
std::string Parser::expectName(const char* what){
    auto t = lex_.next();
    if (t.type != TokenType::NAME)
        throw std::runtime_error(std::string("Expected NAME for ") + what + loc(t));
    return t.lexeme;
}

// キーワードを取得する関数
std::string Parser::expectKeyword(const char* what){
    auto t = lex_.next();
    if (t.type != TokenType::KEYWORD)
        throw std::runtime_error(std::string("Expected KEYWORD for ") + what + loc(t));
    return t.lexeme;
}

// ---関数項と数値式---
// 関数項を解析する関数
FuncTerm Parser::parseFuncTermInParens(){
    lex_.expect(TokenType::LPAR, "(");
    FuncTerm ft;
    ft.name = lex_.expect(TokenType::NAME, "function name").lexeme;
    while (lex_.peek().type != TokenType::RPAR) {
        auto t = lex_.next();
        if (t.type == TokenType::NAME || t.type == TokenType::VARIABLE) {
            ft.args.push_back(t.lexeme);
        } else {
            throw std::runtime_error("term expected (name or variable) in function term" + loc(t));
        }
    }
    lex_.expect(TokenType::RPAR, ")");
    return ft;
}

// 数値式を解析する関数
NumExpr Parser::parseNumericExpr(){
    auto t = lex_.peek();

    // 定数の場合
    if (t.type == TokenType::NUMBER) {
        NumExpr ne; ne.kind = NumExpr::CONST;
        ne.value = std::stod(lex_.next().lexeme);
        return ne;
    }

    // 括弧で始まる場合
    if (t.type == TokenType::LPAR) {
        lex_.expect(TokenType::LPAR, "(");
        auto head = lex_.next(); // NAME("+","*","/") または DASH(" - ")


        // 関数の定義
        auto make_op = [&](NumExpr::Kind k){
            NumExpr ne;
            ne.kind = k;
            while (lex_.peek().type != TokenType::RPAR) {
                ne.args.push_back(parseNumericExpr()); // 再帰
            }
            lex_.expect(TokenType::RPAR, ")");
            return ne;
        };

        // head が演算子である場合
        if (head.type == TokenType::NAME &&
            (head.lexeme == "+" || head.lexeme == "*" || head.lexeme == "/")) {
            if (head.lexeme == "+") return make_op(NumExpr::ADD);
            if (head.lexeme == "*") return make_op(NumExpr::MUL);
            return make_op(NumExpr::DIV); /* "/" */ 
        }
        if (head.type == TokenType::DASH) { // 先頭が '-' のとき
            return make_op(NumExpr::SUB);
        }

        // head が関数名である場合
        if (head.type == TokenType::NAME) {
            FuncTerm ft;
            ft.name = head.lexeme;
            while (lex_.peek().type != TokenType::RPAR) {
                auto a = lex_.next();
                if (a.type == TokenType::NAME || a.type == TokenType::VARIABLE) {
                    ft.args.push_back(a.lexeme);
                } else {
                    throw std::runtime_error(
                        "term expected (name or variable) in function term" + loc(a));
                }
            }
            lex_.expect(TokenType::RPAR, ")");
            NumExpr ne; ne.kind = NumExpr::FUNC; ne.func = std::move(ft);
            return ne;
        }

        throw std::runtime_error("numeric expr head must be + - * / or function name" + loc(head));
    }

    throw std::runtime_error("numeric expr expected (number or '(' ... ')')" + loc(t));
}


// ---再帰下降のための式---
// 命題の解析を行う関数
Atom Parser::parseAtomWithHead(const std::string& head){
    Atom a; a.pred = head;
    while (lex_.peek().type != TokenType::RPAR) {
        auto t = lex_.next();
        if (t.type == TokenType::NAME || t.type == TokenType::VARIABLE) {
            a.args.push_back(t.lexeme);
        } else {
            throw std::runtime_error("term expected (name or variable)"+loc(t));
        }
    }
    lex_.expect(TokenType::RPAR, ")");
    return a;
}

// 左括弧を読んだ後に、parseAtomWithHeadを呼び出す
Atom Parser::parseAtom(){
    lex_.expect(TokenType::LPAR, "(");
    auto head = lex_.expect(TokenType::NAME, "predicate name").lexeme;
    return parseAtomWithHead(head);
}

// 論理式を解析する関数
Formula Parser::parseFormula(){
    // 先頭は '(' 
    lex_.expect(TokenType::LPAR, "(");
    auto head = lex_.next();
    if (head.type == TokenType::NAME && head.lexeme == "and") { // and の場合
        Formula f; 
        f.kind = Formula::AND;
        while (lex_.peek().type != TokenType::RPAR) {
            f.children.push_back(parseFormula()); // 再帰的に読み込む
        }
        lex_.expect(TokenType::RPAR, ")");
        return f;
    } else if (head.type == TokenType::NAME && head.lexeme == "not") { // not の場合
        Formula f; 
        f.kind = Formula::NOT;
        f.child = std::make_unique<Formula>(parseFormula()); // 再帰的に読み込む
        lex_.expect(TokenType::RPAR, ")");
        return f;
    } else if (head.type == TokenType::NAME && head.lexeme == "increase") {
        Formula f; f.kind = Formula::INCREASE;
        // (increase <lvalue:(func-term)> <rhs:(num-expr)>)
        f.inc.lhs = parseFuncTermInParens();
        f.inc.rhs = parseNumericExpr();
        lex_.expect(TokenType::RPAR, ")");
        return f;
    } else if (head.type == TokenType::NAME) {
        // '(' + NAME で始まる => atom
        Formula f; f.kind = Formula::ATOM;
        f.atom = parseAtomWithHead(head.lexeme); // ここでは '(' 済み
        return f;
    } else {
        throw std::runtime_error("formula head must be NAME 'and'/'not'/predicate"+loc(head));
    }
}

// ---var list: (?x ?y - T ?z - U)---
std::vector<TypedVar> Parser::parseVarListInParens(){
    lex_.expect(TokenType::LPAR, "(");
    std::vector<TypedVar> out;
    std::vector<std::string> buf; // ?x ?y などタイプ未確定のバッファ
    while (true){
        auto t = lex_.peek();
        if (t.type == TokenType::RPAR) {
            lex_.next(); // consume ')'
            // 残りがあればobject型として登録する
            for (auto& v : buf) out.push_back({v, "object"});
            buf.clear();
            break;
        }
        t = lex_.next();
        if (t.type == TokenType::VARIABLE) {
            buf.push_back(t.lexeme);
        } else if (t.type == TokenType::DASH) {
            std::string ty = expectName("type name after '-'"); // ty means type
            for (auto& v : buf) out.push_back({v, ty});
            buf.clear();
        } else {
            throw std::runtime_error("variable or '-' expected in var list"+loc(t));
        }
    }
    return out;
}

// ---Domain---
// requirements 部分の解析
std::vector<std::string> Parser::parseRequirementsSection(){
    // 直前で '(:requirements' の ':' は消費済みなので、ここでは本体だけ読む
    std::vector<std::string> r;
    while (lex_.peek().type != TokenType::RPAR) {
        auto k = lex_.expect(TokenType::KEYWORD, "requirement keyword");
        r.push_back(k.lexeme);
    }
    lex_.expect(TokenType::RPAR, ")");
    return r;
}

// 変数のタイプの解析
void Parser::parseTypesSectionInto(Domain& d) {
    std::vector<std::string> buf; // 子候補用のバッファ
    while (lex_.peek().type != TokenType::RPAR) {
        auto t = lex_.next();
        if (t.type == TokenType::NAME) {
            buf.push_back(t.lexeme);
            d.types.push_back(t.lexeme); // 型名リストにも入れる
        } else if (t.type == TokenType::DASH) {
            std::string parent = expectName("super type");
            d.types.push_back(parent);
            for (auto& child : buf) {
                d.supertypes[child].push_back(parent);
            }
            buf.clear();
        } else {
            throw std::runtime_error("unexpected token in :types" + loc(t));
        }
    }
    lex_.expect(TokenType::RPAR, ")");

    // 残った型は明示的な親が無いので object 直下にぶら下げる
    for (auto& child : buf) {
        d.supertypes[child].push_back("object");
    }
    // object 自体が宣言されていない場合のために object も登録する
    d.types.push_back("object");
}

// 述語スキーマの解析
std::vector<PredicateSchema> Parser::parsePredicatesSection(){
    std::vector<PredicateSchema> ps;
    while (lex_.peek().type != TokenType::RPAR) {
        lex_.expect(TokenType::LPAR, "(");
        PredicateSchema s;
        s.name = expectName("predicate name");
        // パラメータ列（'?'と'-'の並び）を閉じ括弧まで読む
        std::vector<std::string> buf;
        while (lex_.peek().type != TokenType::RPAR) {
            auto t = lex_.next();
            if (t.type == TokenType::VARIABLE) {
                buf.push_back(t.lexeme);
            } else if (t.type == TokenType::DASH) {
                std::string ty = expectName("type name");
                for (auto& v : buf) s.params.push_back({v, ty});
                buf.clear();
            } else {
                throw std::runtime_error("variable or '-' expected in predicate params"+loc(t));
            }
        }
        lex_.expect(TokenType::RPAR, ")");

        // 型が付かなかった残りはobject型として入れる
        for (auto& v : buf) s.params.push_back({v, "object"});
        ps.push_back(std::move(s));
    }
    lex_.expect(TokenType::RPAR, ")");
    return ps;
}

// function の解析
std::vector<FunctionSchema> Parser::parseFunctionsSection() {
    std::vector<FunctionSchema> out;

    // :functions の中身は、( ... ) の並び＋任意の "- <type>" 指定が続く形
    // ex1: (:functions (total-cost) - number)
    // ex2: (:functions (distance ?a - loc ?b - loc) - number (fuel) - number)
    // ex3: (:functions (foo ?x ?y) (bar ?z) - number)

    while (lex_.peek().type != TokenType::RPAR) {
        // group に ( ... ) の並びを保存する
        std::vector<FunctionSchema> group;

        // 少なくとも1個の "( ... )" を読む
        do {
            lex_.expect(TokenType::LPAR, "(");
            FunctionSchema fs;
            fs.name = expectName("function name"); // 関数名

            // パラメータ列（'?x ... - T ...'）を ')' まで読む
            std::vector<std::string> buf;
            while (lex_.peek().type != TokenType::RPAR) {
                auto t = lex_.next();
                if (t.type == TokenType::VARIABLE) {
                    buf.push_back(t.lexeme);
                } else if (t.type == TokenType::DASH) {
                    std::string ty = expectName("type name after '-'");
                    for (auto& v : buf) fs.params.push_back({v, ty});
                    buf.clear();
                } else {
                    throw std::runtime_error("variable or '-' expected in function params" + loc(t));
                }
            }
            lex_.expect(TokenType::RPAR, ")");

            // 型が付かなかった残りは object 型
            for (auto& v : buf) fs.params.push_back({v, "object"});

            // 既定の戻り型は "number"
            fs.rettype = "number";

            group.push_back(std::move(fs));

            // 次が '(' なら同じ group に積み増す
            // '(' でなければ break
        } while (lex_.peek().type == TokenType::LPAR);

        // group 直後に "- <type>" が来たら、その型を group 全体へ適用
        if (lex_.peek().type == TokenType::DASH) {
            lex_.next(); // consume '-'
            std::string rt = expectName("function return type name after '-'");
            for (auto& fs : group) fs.rettype = rt;
        }

        // group を out に追加
        for (auto& fs : group) out.push_back(std::move(fs));
    }

    lex_.expect(TokenType::RPAR, ")"); // :functions ブロック終端
    return out;
}


// アクションの解析
Action Parser::parseActionSection(){
    Action a;
    a.name = expectName("action name"); // アクション名

    // :parameters
    if (expectKeyword(":parameters?").compare("parameters") != 0)
        throw std::runtime_error("expected :parameters in action");
    a.params = parseVarListInParens(); // アクションの引数リスト

    // :precondition <formula>
    if (expectKeyword(":precondition?").compare("precondition") != 0)
        throw std::runtime_error("expected :precondition in action");
    a.precond = parseFormula(); // アクションの前提条件

    // :effect <formula>
    if (expectKeyword(":effect?").compare("effect") != 0)
        throw std::runtime_error("expected :effect in action");
    a.effect = parseFormula(); // アクションの効果

    // アクションの閉じ括弧は呼び出し元が読む
    return a;
}

// 型付定数の解析
std::vector<std::pair<std::string,std::string>> Parser::parseConstantsSection(){
    std::vector<std::pair<std::string,std::string>> cs; // 定数と型を保存するベクトル, <name, type>
    std::vector<std::string> buf; // NAME 一時保存用のバッファ

    while (lex_.peek().type != TokenType::RPAR) { // 右括弧に到達するまで
        auto t = lex_.next();
        if (t.type == TokenType::NAME) {
            buf.push_back(t.lexeme);
        } else if (t.type == TokenType::DASH) {
            std::string ty = expectName("type name after '-'");
            for (auto& n : buf) cs.push_back({n, ty});
            buf.clear(); // バッファの解放
        } else {
            throw std::runtime_error("NAME or '-' expected in :constants");
        }
    }
    lex_.expect(TokenType::RPAR, ")");
    // 最後に残ったものは, object 型とする
    for (auto& n : buf) cs.push_back({n, "object"});
    return cs;
}

// ドメインの解析
Domain Parser::parseDomain(){
    Domain d;

    // (define (domain NAME) ... ) の形を予想して読み込む
    lex_.expect(TokenType::LPAR, "(");
    if (expectName("'define'").compare("define") != 0)
        throw std::runtime_error("expected define");
    lex_.expect(TokenType::LPAR, "(");
    if (expectName("'domain'").compare("domain") != 0)
        throw std::runtime_error("expected (domain NAME)");
    d.name = expectName("domain name");
    lex_.expect(TokenType::RPAR, ")");

    // セクションを読む
    while (true) {
        auto t = lex_.peek();

        if (t.type == TokenType::RPAR) { lex_.next(); break; } // domainの解析を終える

        lex_.expect(TokenType::LPAR, "(");
        auto kw = expectKeyword("section keyword");

        // :requirements
        if (kw == "requirements") {
            d.requirements = parseRequirementsSection();
            continue;
        }

        // :types
        if (kw == "types") {
            parseTypesSectionInto(d);
            continue;
        }

        // :predicates
        if (kw == "predicates") {
            d.predicates = parsePredicatesSection();
            continue;
        }

        // :functions
        if (kw == "functions") {
            d.functions = parseFunctionsSection();
            continue;
        }

        // :action
        if (kw == "action") {
            Action a = parseActionSection();
            lex_.expect(TokenType::RPAR, ")"); // :action ブロックの ')'
            d.actions.push_back(std::move(a));
            continue;
        }

        // :constants
        if (kw == "constants") {
            d.constants = parseConstantsSection();
            continue;
        }

        // 未対応セクションは、 ) までスキップする。ただし、セクションの追加を後にする場合は、ここに追加する
        while (lex_.peek().type != TokenType::RPAR) (void)lex_.next();
        lex_.expect(TokenType::RPAR, ")");
    }

    // :types セクションがない場合でも、 "object" を最低限登録する
    {
        bool has_object = false;
        for (const auto& ty : d.types) {
            if (ty == "object") {
                has_object = true;
                break;
            }
        }
        if (!has_object) {
            d.types.push_back("object");
        }
    }

    // d.types に type がなければ追加し、親を object にする関数
    auto ensure_type = [&](const std::string& ty){
        if (ty.empty()) return;
        bool present = false;
        for (const auto& t : d.types) {
            if (t == ty) {
                present = true;
                break;
            }
        }
        if (!present) {
            d.types.push_back(ty);
            d.supertypes[ty].push_back("object");
        }
    };

    // 述語パラメータで使われた型を登録する
    for (const auto& ps : d.predicates) {
        for (const auto& tv : ps.params) {
            ensure_type(tv.type);
        }
    }

    // アクションパラメータで使われた型を登録する
    for (const auto& a : d.actions) {
        for (const auto& tv : a.params) {
            ensure_type(tv.type);
        }
    }

    // 関数パラメータ / 戻り型で使われた型を登録する
    for (const auto& fs : d.functions) {
        for (const auto& tv : fs.params) {
            ensure_type(tv.type);
        }
        ensure_type(fs.rettype); // 通常 number
    }

    // d.types を重複排除する
    {
        std::vector<std::string> uniq;
        uniq.reserve(d.types.size());
        std::unordered_set<std::string> seen;
        for (const auto& ty : d.types) {
            if (seen.insert(ty).second) {
                uniq.push_back(ty);
            }
            d.types.swap(uniq);
        }
    }

    return d;
}

// ---Problem---
// objects セクションの解析
std::vector<std::pair<std::string,std::string>> Parser::parseObjectsSection(){
    std::vector<std::pair<std::string,std::string>> objs;
    std::vector<std::string> buf;
    while (lex_.peek().type != TokenType::RPAR) {
        auto t = lex_.next();
        if (t.type == TokenType::NAME) {
            buf.push_back(t.lexeme);
        } else if (t.type == TokenType::DASH) {
            std::string ty = expectName("type name");
            for (auto& n : buf) objs.push_back({n, ty});
            buf.clear();
        } else {
            throw std::runtime_error("NAME or '-' expected in :objects"+loc(t));
        }
    }
    lex_.expect(TokenType::RPAR, ")");
    // 残りはobject型として登録する
    for (auto& n : buf) objs.push_back({n, "object"});
    return objs;
}

// init セクションの解析
std::vector<Atom> Parser::parseInitSection(){
    std::vector<Atom> init;
    while (lex_.peek().type != TokenType::RPAR) {
        init.push_back(parseAtom()); // init はアトム列（andやnotを使わない）
    }
    lex_.expect(TokenType::RPAR, ")");
    return init;
}

// init セクションの解析 (数値 init が含まれている init 用)
void Parser::parseInitSectionInto(Problem& p) {
    while (lex_.peek().type != TokenType::RPAR) {
        lex_.expect(TokenType::LPAR, "(");
        auto head = lex_.next();

        // (and ...) の場合
        if (head.type == TokenType::NAME && head.lexeme == "and") {
            while (lex_.peek().type != TokenType::RPAR) {
                lex_.expect(TokenType::LPAR, "(");
                auto h2 = lex_.next();
                // (= <func-term> <number>) の場合
                if (h2.type == TokenType::NAME && h2.lexeme == "=") {
                    NumericInit ni;
                    ni.lhs = parseFuncTermInParens();
                    NumExpr rhs = parseNumericExpr();
                    if (rhs.kind != NumExpr::CONST) {
                        throw std::runtime_error("RHS of numeric init must be a number");
                    }
                    ni.value = rhs.value;
                    lex_.expect(TokenType::RPAR, ")");
                    p.init_num.push_back(std::move(ni));
                } else if (h2.type  == TokenType::NAME) { // 命題の場合
                    Atom a = parseAtomWithHead(h2.lexeme);
                    p.init.push_back(std::move(a));
                } else {
                    throw std::runtime_error("invalid init item head");
                }
            }
            lex_.expect(TokenType::RPAR, ")");
            continue;
        }

        // (= <func-term> <number>) の場合
        if (head.type == TokenType::NAME && head.lexeme == "=") {
            NumericInit ni;
            ni.lhs = parseFuncTermInParens();   // ex: (total-cost)
            NumExpr rhs = parseNumericExpr();   // ex: 0
            if (rhs.kind != NumExpr::CONST)
                throw std::runtime_error("RHS of numeric init must be a number" + loc(lex_.peek()));
            ni.value = rhs.value;
            lex_.expect(TokenType::RPAR, ")");
            p.init_num.push_back(std::move(ni));
            continue;
        }

        // 命題の場合
        if (head.type == TokenType::NAME) {
            Atom a = parseAtomWithHead(head.lexeme);
            p.init.push_back(std::move(a));
            continue;
        }

        throw std::runtime_error("invalid init item head" + loc(head));
    }
    lex_.expect(TokenType::RPAR, ")");
}

// metric セクションの解析
void Parser::parseMetricSectionInto(Problem& p){
    // (:metric minimize <num-expr>) / (:metric maximize <num-expr>)
    auto senseName = lex_.expect(TokenType::NAME, "minimize/maximize").lexeme;
    if (senseName == "minimize")      p.metric.sense = Metric::MINIMIZE;
    else if (senseName == "maximize") p.metric.sense = Metric::MAXIMIZE;
    else throw std::runtime_error("metric sense must be 'minimize' or 'maximize'");

    p.metric.expr = parseNumericExpr();
    p.metric.present = true;

    lex_.expect(TokenType::RPAR, ")"); // :metric の閉じ括弧 (右括弧)
}

// 問題の解析
Problem Parser::parseProblem(){
    Problem p;

    // (define (problem NAME) (:domain NAME) ... ) の冒頭部分を解析する
    lex_.expect(TokenType::LPAR, "(");
    if (expectName("'define'").compare("define") != 0)
        throw std::runtime_error("expected define");
    lex_.expect(TokenType::LPAR, "(");
    if (expectName("'problem'").compare("problem") != 0)
        throw std::runtime_error("expected (problem NAME)");
    p.name = expectName("problem name");
    lex_.expect(TokenType::RPAR, ")");

    // 必須: (:domain NAME)
    lex_.expect(TokenType::LPAR, "(");
    if (expectKeyword(":domain").compare("domain") != 0)
        throw std::runtime_error("expected :domain");
    p.domain_name = expectName("domain name");
    lex_.expect(TokenType::RPAR, ")");

    // 残りセクション
    while (true) {
        auto t = lex_.peek();
        if (t.type == TokenType::RPAR) { lex_.next(); break; } // problem 閉じ
        lex_.expect(TokenType::LPAR, "(");
        auto kw = expectKeyword("problem section");

        // :objects
        if (kw == "objects") {
            p.objects = parseObjectsSection();
            continue;
        }

        // :init
        if (kw == "init") {
            parseInitSectionInto(p);
            continue;
        }

        // :goal
        if (kw == "goal") {
            p.goal = parseFormula();
            lex_.expect(TokenType::RPAR, ")");
            continue;
        }

        // :metric
        if (kw == "metric") {
            parseMetricSectionInto(p);
            continue;
        }

        // 未対応セクションは ) までスキップ
        while (lex_.peek().type != TokenType::RPAR) (void)lex_.next();
        lex_.expect(TokenType::RPAR, ")");
    }

    return p;
}



} // namespace planner
