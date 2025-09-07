#include "grounding.hpp"
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace planner {

// --- 内部で使う補助関数 ---
// サブタイプか判定する関数
static bool is_subtype(const Domain& d, const std::string& child, const std::string& want) {
    if (child == want) return true;
    std::vector<std::string> stack = {child};
    std::unordered_set<std::string> seen;
    while (!stack.empty()) {
        std::string cur = stack.back(); stack.pop_back();
        if (!seen.insert(cur).second) continue;
        auto it = d.supertypes.find(cur);
        if (it == d.supertypes.end()) continue;
        for (auto& p : it->second) { // it: (child, parents)
            if (p == want) return true;
            stack.push_back(p);
        }
    }
    return false; // サブタイプではない場合
}

// 関数名と引数の組み合わせを string 形式に変換する関数
static std::string func_key(const std::string& name, const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << name << "(";
    for (size_t i=0;i<args.size();++i){
        if (i) oss << ",";
        oss << args[i];
    }
    oss << ")";
    return oss.str();
}


// 述語スキーマを端から検索する関数
static const PredicateSchema& find_pred(const Domain& d, const std::string& name) {
    for (const auto& ps : d.predicates) {
        if (ps.name == name) return ps;
    }
    throw std::runtime_error("unknown predicate: " + name);
}

// 関数スキーマを端から検索する関数
static const FunctionSchema* try_find_func(const Domain& d, const std::string& name) {
    for (const auto& fs : d.functions) {
        if (fs.name == name) return &fs;
    }
    return nullptr;
}

// and で繋がれた precondition から、リテラルを集める関数
static void collect_literals_pre(const Formula& f,
                                 std::vector<Atom>& pos,
                                 std::vector<Atom>& neg)
{
    if (f.kind == Formula::ATOM) { // 命題
        pos.push_back(f.atom);
        return;
    }
    if (f.kind == Formula::NOT) { // Not (命題)
        if (!f.child || f.child->kind != Formula::ATOM) // not の対象先が atom 出ない場合は除外
            throw std::runtime_error("NOT must wrap an atom in precondition");
        neg.push_back(f.child->atom);
        return;
    }
    if (f.kind == Formula::AND) { // And (...) (...)
        for (auto& c : f.children) collect_literals_pre(c, pos, neg);
        return;
    }
    if (f.kind == Formula::INCREASE) { // increase が含まれる場合はエラー
        throw std::runtime_error("increase not allowed in precondition");
    }
    // どのノードにも該当しない場合はエラー
    throw std::runtime_error("unsupported formula node in precondition");
}

// and で繋がれた effect から、リテラルを集める関数 (基本的には、collect_literals_pre と同様の処理)
static void collect_effects(const Formula& f,
                            std::vector<Atom>& add,
                            std::vector<Atom>& del,
                            std::vector<Formula::Increase>& incs)
{
    if (f.kind == Formula::ATOM) { // 命題の場合
        add.push_back(f.atom);
        return;
    }
    if (f.kind == Formula::NOT) { // Not (命題)
        if (!f.child || f.child->kind != Formula::ATOM) // not の対象先が atom 出ない場合は除外
            throw std::runtime_error("NOT must wrap an atom in effect");
        del.push_back(f.child->atom);
        return;
    }
    if (f.kind == Formula::AND) { // And (...) (...), 再帰的に処理
        for (auto& c : f.children) collect_effects(c, add, del, incs);
        return;
    }
    if (f.kind == Formula::INCREASE) { // increase が含まれる場合はエラー
        incs.push_back(f.inc);
        return;
    }
    // サポートされていないノードの場合はエラー
    throw std::runtime_error("unsupported formula node in effect");
}

// 代入 σ: var->obj を Atom へ適用
static Atom subst_atom(const Atom& a, const std::unordered_map<std::string,std::string>& sigma) {
    Atom b;
    b.pred = a.pred;
    b.args.reserve(a.args.size());
    for (auto& s : a.args) {
        auto it = sigma.find(s);
        if (it != sigma.end()) {
            b.args.push_back(it->second);
        } else {
            b.args.push_back(s);
        }
    }
    return b;
}

// 変数リスト var -> type の置換 σ を生成する関数
static std::unordered_map<std::string,std::string>
var_types(const std::vector<TypedVar>& vs) {
    std::unordered_map<std::string,std::string> out;
    for (auto& v : vs) out[v.name] = v.type;
    return out;
}

// オブジェクトと、要求した型が適合するかどうかを判定する関数 
static bool object_fits_type(const Domain& d,
                             const std::unordered_map<std::string,std::string>& obj_ty,
                             const std::string& obj, // オブジェクト
                             const std::string& need_ty) // 必要な型
{
    auto it = obj_ty.find(obj);
    if (it == obj_ty.end()) return false;
    return is_subtype(d, it->second, need_ty); // 型適合性のチェック
}

// Atom -> GroundAtom 変換を行う関数
static GroundAtom ground_atom(const Atom& a,
                              const Domain& d,
                              const std::unordered_map<std::string,int>& obj_id, // object -> id
                              const std::unordered_map<std::string,std::string>& obj_ty, // object -> type
                              const std::unordered_map<std::string,int>& pred_id) // predicate -> id
{
    auto pit = pred_id.find(a.pred);
    if (pit == pred_id.end()) // マップで見つからなかった場合はエラー
        throw std::runtime_error("predicate not declared: " + a.pred);
    int pid = pit->second; // predicate id
    const auto& ps = find_pred(d, a.pred);
    if (ps.params.size() != a.args.size()) // ドメインで宣言された述語の引数の数と異なる場合はエラー
        throw std::runtime_error("arity mismatch in atom: " + a.pred);

    GroundAtom ga;
    ga.pred = pid; // id 設定
    ga.args.reserve(a.args.size());
    for (size_t i=0;i<a.args.size();++i) {
        const std::string& obj = a.args[i];
        auto oit = obj_id.find(obj); // object id
        if (oit == obj_id.end()) // object が見つからなかった場合はエラー
            throw std::runtime_error("unknown object: " + obj);
        // 型チェック（述語パラメータ型に対して）
        if (!object_fits_type(d, obj_ty, obj, ps.params[i].type))
            throw std::runtime_error("object type mismatch for " + obj + " in " + a.pred);
        ga.args.push_back(oit->second);
    }
    return ga;
}

// NumExpr の評価を行う関数（関数値は Problem の init_num から定まるとし、未定義は 0 とする）
static double eval_numeric(const NumExpr& ne,
                           const std::unordered_map<std::string,double>& func_values,
                           const std::unordered_map<std::string,std::string>& sigma) // var->obj（FuncTerm 用）
{
    using K = NumExpr::Kind; // 列挙型のエイリアス
    switch (ne.kind) {
        case K::CONST: { // 定数の場合
            return ne.value;
        }
        case K::FUNC: { // 関数の場合
            std::vector<std::string> args;
            args.reserve(ne.func.args.size());
            for (auto& a : ne.func.args) {
                auto it = sigma.find(a);
                if (it != sigma.end()) {
                    args.push_back(it->second);
                } else {
                    args.push_back(a);
                }
            }
            std::string key = func_key(ne.func.name, args);
            auto it = func_values.find(key);
            return (it == func_values.end()) ? 0.0 : it->second; // 関数の id を返す
        }
        case K::ADD: { // 加算の場合
            double s = 0.0;
            for (auto& a : ne.args) s += eval_numeric(a, func_values, sigma);
            return s;
        }
        case K::MUL: { // 乗算の場合
            double p = 1.0;
            for (auto& a : ne.args) p *= eval_numeric(a, func_values, sigma);
            return p;
        }
        case K::SUB: { // 減算の場合
            if (ne.args.empty()) return 0.0;
            double x = eval_numeric(ne.args[0], func_values, sigma);
            if (ne.args.size() == 1) return -x;
            for (size_t i=1;i<ne.args.size();++i) x -= eval_numeric(ne.args[i], func_values, sigma);
            return x;
        }
        case K::DIV: { // 除算の場合
            if (ne.args.size() != 2)
                throw std::runtime_error("division expects 2 args");
            double a = eval_numeric(ne.args[0], func_values, sigma);
            double b = eval_numeric(ne.args[1], func_values, sigma);
            return a / b;
        }
    }
    return 0.0;
}

// GroundAtom を文字列に変換する関数
std::string to_string(const GroundAtom& ga, const GroundTask& gt) {
    std::ostringstream oss;
    const auto& ps = gt.preds[ga.pred];
    oss << "(" << ps.name;
    for (size_t i=0;i<ga.args.size();++i) {
        int oid = ga.args[i];
        oss << " " << gt.objects[oid];
    }
    oss << ")";
    return oss.str();
}

// GroundAtom のキーを生成する関数
inline std::string key_of(const GroundAtom& ga) {
    std::ostringstream oss;
    oss << ga.pred << ":";
    for (int id : ga.args) {
        oss << id << ",";
    }
    return oss.str();
}

// pred id とオブジェクト id 群から R+/init 用キーを直で作る（ground_atom を呼ばない）
static inline std::string key_of_ids(int pid, const std::vector<int>& ids) {
    std::ostringstream oss;
    oss << pid << ":";
    for (int id : ids) oss << id << ",";
    return oss.str();
}


// ---グラウンド化を行う主要関数---

GroundTask ground(const Domain& d, const Problem& p)
{
    GroundTask G;

    // problem の objects の名前と型を登録する
    for (auto& [name, ty] : p.objects) {
        if (G.obj_id.count(name)) throw std::runtime_error("duplicate object: " + name);
        int id = static_cast<int>(G.objects.size());
        G.objects.push_back(name); // オブジェクト名を登録
        G.obj_id[name] = id; // object -> id
        G.obj_ty[name] = ty; // object -> type
    }

    // predicates
    for (auto& ps : d.predicates) {
        if (G.pred_id.count(ps.name)) throw std::runtime_error("duplicate predicate: " + ps.name);
        int id = static_cast<int>(G.preds.size());
        G.pred_id[ps.name] = id;
        GroundTask::PredSchema s;
        s.name = ps.name;
        for (auto& tv : ps.params) s.types.push_back(tv.type);
        G.preds.push_back(std::move(s));
    }

    // functions（コスト評価用）
    for (auto& fs : d.functions) {
        if (!G.func_id.count(fs.name)) {
            int id = static_cast<int>(G.funcs.size());
            G.func_id[fs.name] = id;
            GroundTask::FuncSchema s;
            s.name = fs.name;
            for (auto& tv : fs.params) s.types.push_back(tv.type);
            G.funcs.push_back(std::move(s));
        }
    }
    // functions の初期値（Problem の init_num から）を func_values に登録する
    for (auto& ini : p.init_num) {
        std::string key = func_key(ini.lhs.name, ini.lhs.args);
        G.func_values[key] = ini.value;
    }

    // init (命題) を ground 化し、 G.init_pos に登録する
    for (auto& a : p.init) {
        G.init_pos.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));
    }

    // init のハッシュ集合（静的述語チェックで使う）を一度だけ構築
    std::unordered_set<std::string> init_set;
    init_set.reserve(G.init_pos.size() * 2);
    for (auto& f : G.init_pos) init_set.insert(key_of(f));

    // goal
    {
        std::vector<Atom> gp, gn;
        collect_literals_pre(p.goal, gp, gn); // positive, negative のリテラルをそれぞれ集める
        for (auto& a : gp) G.goal_pos.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));
        for (auto& a : gn) G.goal_neg.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));
    }

    // --- 静的述語の検出 ---
    // 使用するデータ構造
    std::vector<bool> is_dynamic;
    is_dynamic.resize(G.preds.size(), false);

    // 動的述語をマークするためのラムダ関数
    auto mark_dynamic_from = [&](const Formula& eff){
        std::vector<Atom> add, del;
        std::vector<Formula::Increase> incs;
        collect_effects(eff, add, del, incs); 
        auto mark = [&](const Atom& a){
            auto it = G.pred_id.find(a.pred);
            if (it != G.pred_id.end()) is_dynamic[it->second] = true; // 見つかった場合は動的にする
        };
        for (auto& a : add) mark(a);
        for (auto& a : del) mark(a);
    };


    // effect に基づいて動的述語をマークする
    for (const auto& act : d.actions) mark_dynamic_from(act.effect);

    // 静的かどうかテストするラムダ関数
    auto is_static_pred = [&](int pid){ return !is_dynamic[pid]; }; // 述語 id を受け取り、vector<bool> を参照して静的かどうかを判定する


    // actions
    for (const auto& act : d.actions) {
        // テンプレートとしての pre/effect を集める
        std::vector<Atom> preP_tmpl, preN_tmpl, effA_tmpl, effD_tmpl;
        std::vector<Formula::Increase> incs_tmpl;
        collect_literals_pre(act.precond, preP_tmpl, preN_tmpl);
        collect_effects(act.effect, effA_tmpl, effD_tmpl, incs_tmpl);

        // 各パラメータに適合するオブジェクト候補集合
        std::vector<std::vector<std::string>> cand; // 各パラメータに対するオブジェクト候補のリストが順に入る
        cand.resize(act.params.size());
        for (size_t i=0; i<act.params.size(); ++i) {
            const auto& tv = act.params[i];
            for (auto& [oname, oty] : G.obj_ty) {
                if (object_fits_type(d, G.obj_ty, oname, tv.type)) {
                    cand[i].push_back(oname); // type に適合するオブジェクトを追加する
                }
            }
            if (cand[i].empty()) {
                // このパラメータに合う object がないならばアクションは生成されないので、候補をクリアする
                cand.clear();
                break;
            }
        }
        if (cand.empty() && !act.params.empty()) continue;

        // 直積で全代入を生成
        std::vector<size_t> idx(act.params.size(), 0);
        auto vars = var_types(act.params); // 各パラメータと型のマップ

        // sigma の展開、先に早期チェックを行うラムダ関数
        auto make_ground_action = [&](const std::unordered_map<std::string,std::string>& sigma) {
            G.stats.candidates++; // 候補カウント

            // --- all-different による早期チェック ---
            //if (act.params.size() > 1) {
            //    std::unordered_set<std::string> seen;
            //    for (auto& tv : act.params) {
            //        const std::string& obj = sigma.at(tv.name);
            //        if (!seen.insert(obj).second) { // すでに出た object ならば 
            //            G.stats.by_typing_allDiff++;
            //            return; 
            //        }
            //    }
            //}

            // --- 静的述語の早期チェック ---
            auto is_static_atom = [&](const Atom& a)->bool {
                auto it = G.pred_id.find(a.pred);
                return (it != G.pred_id.end()) && is_static_pred(it->second); // 述語が見つかり、かつ静的なら true
            };

            auto holds_in_init = [&](const Atom& a_after_subst, bool positive)->bool {
                auto pit = G.pred_id.find(a_after_subst.pred); // 述語の ID を取得
                if (pit == G.pred_id.end()) return false; // 未宣言述語ならば false
                std::vector<int> ids;
                ids.reserve(a_after_subst.args.size());
                for (auto& obj : a_after_subst.args) {
                    auto oit = G.obj_id.find(obj);
                    if (oit == G.obj_id.end()) return false;
                    ids.push_back(oit->second);
                }
                bool in_init = init_set.count(key_of_ids(pit->second, ids));
                return positive ? in_init : !in_init; // positive の場合は in_init を返し、そうでなければ !in_init を返す
            };

            // preP_tmpl / preN_tmpl のうち静的なものだけ eval
            for (const auto& a : preP_tmpl) {
                if (!is_static_atom(a)) continue;
                if (!holds_in_init(subst_atom(a, sigma), /*positive=*/true)) { // positive なので true を渡す
                    G.stats.by_static++;
                    return;
                }
            }
            for (const auto& a : preN_tmpl) {
                if (!is_static_atom(a)) continue;
                if (!holds_in_init(subst_atom(a, sigma), /*positive=*/false)) { // negative なので false を渡す
                    G.stats.by_static++;
                    return;
                }
            }

            // --- ここまで通過した sigma のみ、初めて ground 化を行う ---
            GroundAction ga;
            {
                std::ostringstream nm;
                nm << "(" << act.name;
                for (size_t i=0; i<act.params.size(); ++i) {
                    nm << " " << sigma.at(act.params[i].name);  // 空白で区切る
                }
                nm << ")";
                ga.name = nm.str();
            }

            // pre/effect は テンプレ を 代入 してから ground_atom に渡す
            std::vector<Atom> preP = preP_tmpl, preN = preN_tmpl, effA = effA_tmpl, effD = effD_tmpl;

            // 代入、置換によって各リテラルを更新
            for (auto& a : preP) a = subst_atom(a, sigma);
            for (auto& a : preN) a = subst_atom(a, sigma);
            for (auto& a : effA) a = subst_atom(a, sigma);
            for (auto& a : effD) a = subst_atom(a, sigma);

            // Ground 化する
            for (auto& a : preP) ga.pre_pos.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));
            for (auto& a : preN) ga.pre_neg.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));
            for (auto& a : effA) ga.eff_add.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));
            for (auto& a : effD) ga.eff_del.push_back(ground_atom(a, d, G.obj_id, G.obj_ty, G.pred_id));

            // cost（incs_tmpl をそのまま使う）
            double cost = 0.0;
            for (const auto& inc : incs_tmpl) {
                if (inc.lhs.name == "total-cost") {
                    // 変数名とオブジェクト名の対応を作成
                    std::unordered_map<std::string,std::string> var2obj;
                    for (auto& tv : act.params) var2obj[tv.name] = sigma.at(tv.name);
                    cost += eval_numeric(inc.rhs, G.func_values, var2obj); // 置換に基づいて評価する
                }
            }
            ga.cost = cost;
            G.actions.push_back(std::move(ga));
        };


        if (act.params.empty()) { // パラメータがない場合は、置換なしで出力する
            make_ground_action({}); 
        } else {
            while (true) { // 直積ループを行う
                std::unordered_map<std::string,std::string> sigma;
                for (size_t i=0;i<act.params.size();++i) {
                    sigma[act.params[i].name] = cand[i][idx[i]];
                }
                make_ground_action(sigma);

                // 次の組み合わせへ
                size_t k = act.params.size();
                while (k>0) {
                    --k; // index に合わせて、 k を最初に 1 だけ減らす
                    if (++idx[k] < cand[k].size()) break; // もし k 番目の index が範囲内ならば、ここでループを抜ける
                    idx[k] = 0; // 桁が溢れたら 0 に戻す
                }
                if (k==0 && idx[0]==0) break; // すべての桁が溢れたら終了
            }
        }
    }

        

    // --- 前向き到達可能性による pruning ---
    // R+ を init から開始
    std::unordered_set<std::string> R;
    for (auto& f : G.init_pos) R.insert(key_of(f)); // init の positive な事実を R+ に追加

    // R+ の拡張を固定点まで行う
    bool changed = true;
    while (changed) {
        changed = false;
        for (const auto& a : G.actions) {
            bool ok = true;
            for (const auto& pr : a.pre_pos) {
                if (!R.count(key_of(pr))) { // pre_pos の全ての要素が R+ に含まれているかチェック
                    ok = false;
                    break;
                }
            }
            if (!ok) continue; // もしそのアクションの前提条件が満たされていなかったらスキップ
            for (const auto& ad : a.eff_add) {
                if (R.insert(key_of(ad)).second) { // 新しい事実が R+ に追加されたら
                    changed = true;
                }
            }
        }
    }

    // R+ で満たせない action を剪定する
    std::vector<GroundAction> kept;
    kept.reserve(G.actions.size());
    for (auto& a : G.actions) {
        bool ok = true;
        for (auto& pr : a.pre_pos) {
            if (!R.count(key_of(pr))) {
                ok = false;
                break;
            }
        }
        if (ok) {
            kept.push_back(std::move(a));
        } else {
            G.stats.by_forward++;
        }
    }
    G.actions.swap(kept);

    // --- 後ろ向き関連性による pruning ---
    // G（有益事実集合）をゴールから開始
    std::unordered_set<std::string> Gfacts;
    for (auto& g : G.goal_pos) { // ゴールの positive な事実を G に追加
        Gfacts.insert(key_of(g));
    }

    // relevance の固定点計算
    std::vector<char> relevant(G.actions.size(), 0);
    bool grown = true; // G が成長したかどうかのフラグ
    while (grown) {
        grown = false;
        for (std::size_t i=0; i<G.actions.size(); ++i) {
            auto& a = G.actions[i];
            bool produces_goal = false;
            for (auto& ad : a.eff_add) {
                if (Gfacts.count(key_of(ad))) { // add に goal を生み出すものがあるなら
                    produces_goal = true;
                    break;
                }
            }
            if (!produces_goal) continue; // ないならスキップ
            if (!relevant[i]) { // もしそのアクションがまだ relevant でなかったら
                relevant[i] = 1;
                grown = true;
            }
            for (auto& pr : a.pre_pos) {
                if (Gfacts.insert(key_of(pr)).second) { // pre を G に追加
                    grown = true;
                }
            }
        }
    }

    // relevant==1 のみ残す
    std::vector<GroundAction> kept2;
    kept2.reserve(G.actions.size());
    for (std::size_t i=0; i<G.actions.size(); ++i) {
        if (relevant[i]) {
            kept2.push_back(std::move(G.actions[i]));
        } else {
            G.stats.by_backward++;
        }
    }
    G.actions.swap(kept2);

    return G;
}

} // namespace planner
