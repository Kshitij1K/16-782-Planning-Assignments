// clang-format off

#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <stdexcept>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;

public:
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() {
      return initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() {
      return goal_conditions;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}
// clang-format on

typedef unordered_set<GroundedCondition, GroundedConditionHasher,
                      GroundedConditionComparator>
    State;

struct Node {
  Node() : causing_action("", list<std::string>()){};

  State state;
  State parent;
  GroundedAction causing_action;
  double f_val = __DBL_MAX__;
  double g_val = __DBL_MAX__;
  bool is_expanded = false;
};

typedef std::shared_ptr<Node> NodePtr;

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) { return (A->f_val > B->f_val); }
};

class StateHash {
public:
  std::size_t operator()(const State &s) const noexcept {
    std::string string_of_conditions;

    for (auto it : s) {
      string_of_conditions.append(it.toString());
    }

    std::sort(string_of_conditions.begin(), string_of_conditions.end());

    return std::hash<std::string>{}(string_of_conditions);
  }
};

class Tree {
public:
  // void addNode(const State& s, NodePtr data);
  Tree(Env *env, const State &start, const State &goal)
      : start_state_(start), goal_state_(goal) {

    NodePtr start_node = std::make_shared<Node>();
    start_node->state = start_state_;
    start_node->g_val = 0;
    start_node->f_val = heuristicFunction(start_state_);
    // GroundedAction dummy_action = (*env->actions.begin()).get_preconditions;
    setParent(start_state_, start_state_, GroundedAction("", list<string>()));

    for (Action it : env->actions) {
      auto effects = it.get_effects();
      long int literals_added = 0;

      for (auto effect : effects) {
        if (effect.get_truth()) {
          literals_added++;
        }
      }

      if (literals_added < min_literals_added) {
        min_literals_added = literals_added;
      }
    }
  }

  void setParent(const State &child, const State &new_parent,
                 const GroundedAction &new_causing_action) {
    // int j = 8;

    auto node_itr = graph_.find(child);

    if (node_itr == graph_.end()) {
      NodePtr node = std::make_shared<Node>();
      node->state = child;
      node->parent = new_parent;
      node->g_val = graph_.at(new_parent)->g_val + 1;
      node->f_val = node->g_val + heuristicFunction(child);
      node->causing_action = new_causing_action;
      graph_.insert({child, node});
    } else {
      node_itr->second->causing_action = new_causing_action;
      node_itr->second->parent = new_parent;
      node_itr->second->g_val = graph_.at(new_parent)->g_val + 1;
      node_itr->second->f_val =
          node_itr->second->g_val + heuristicFunction(child);
    }
  }
  // void setCost(const State &s, long int g);

  NodePtr getNodeData(const State &s) { return graph_.at(s); }

  list<GroundedAction> constructPlan(const State &reached_goal_state) {
    list<GroundedAction> plan;
    auto curr_node_itr = graph_.find(reached_goal_state);

    while (curr_node_itr->second->causing_action.get_name() == "") {
      plan.push_front(curr_node_itr->second->causing_action);
      curr_node_itr = graph_.find(curr_node_itr->second->parent);
    }

    return plan;
  }

private:
  double heuristicFunction(const State &s) {
    double untrue_conditions = 0.0;
    for (auto condition : goal_state_) {
      if (s.find(condition) == s.end()) {
        untrue_conditions += 1.0;
      }
    }

    return (untrue_conditions) / min_literals_added;
  }

  const State start_state_;
  const State goal_state_;

  long int min_literals_added = INT64_MAX;

  std::unordered_map<State, NodePtr, StateHash> graph_;
};

typedef std::set<NodePtr, NodeComparator> OpenList;

// list<list<int>> generatePermutations(int array_len, int perm_length) {}

// list<GroundedAction> validActions(Env *env, const State &s) {
//   list<GroundedAction> valid_actions;

//   for (auto action : env->actions) {
//   }
// }

void deleteCondition(State &s, GroundedCondition cond_to_delete);
void addCondition(State &s, GroundedCondition cond_to_add);
bool areConditionsSatisfied(const State &s, const State &conditions) {
  for (auto condition : conditions) {
    if (s.find(condition) == s.end()) {
      return false;
    }
  }

  return true;
}

list<string> getNewGroundedArgs(list<string> original_args,
                                list<string> original_grounded_args,
                                list<string> new_args) {
  list<string> new_grounded_args;

  for (auto arg : new_args) {
    auto it = std::find(original_args.begin(), original_args.end(), arg);

    int idx = std::distance(it, original_args.begin());

    new_grounded_args.push_back(
        *std::next(original_grounded_args.begin(), idx));
  }

  return new_grounded_args;
}

bool isActionValid(Env *env, const State &s,
                   const GroundedAction &grounded_action); //{
//   Action action = env->get_action(grounded_action.get_name());

//   for (auto precondition:action.get_preconditions()) {

//   }
// }

void generateActionArgs(Env *env, list<string>::iterator curr_itr,
                        list<string> &args_permutation,
                        list<list<string>> &args_list) {
  if (curr_itr == args_permutation.end()) {
    args_list.push_back(args_permutation);
    return;
  }

  for (auto symbol : env->get_symbols()) {
    *curr_itr = symbol;
    generateActionArgs(env, std::next(curr_itr), args_permutation, args_list);
  }
}

list<GroundedAction> generateValidActions(Env *env, const State &s) {
  list<GroundedAction> valid_actions;

  for (auto action : env->actions) {
    list<list<string>> args_list;
    list<string> args_permutation(action.get_args().size(), "");
    generateActionArgs(env, args_permutation.begin(), args_permutation,
                       args_list);

    for (auto args : args_list) {
      GroundedAction grounded_action(action.get_name(), args);
      if (isActionValid(env, s, grounded_action)) {
        valid_actions.push_back(grounded_action);
      }
    }
  }

  return valid_actions;
}

State getNextState(Env *env, const State &curr_state,
                   const GroundedAction &grounded_action) {

  auto action_args = env->get_action(grounded_action.get_name()).get_args();
  auto grounded_action_args = grounded_action.get_arg_values();

  State result = curr_state;

  auto effects = env->get_action(grounded_action.get_name()).get_effects();
  for (auto effect : effects) {
    list<string> grounded_condition_args = getNewGroundedArgs(
        action_args, grounded_action_args, effect.get_args());

    GroundedCondition gc(effect.get_predicate(), grounded_condition_args);

    if (effect.get_truth()) {
      addCondition(result, gc);
    } else {
      deleteCondition(result, gc);
    }
  }

  return result;
}

list<GroundedAction> planner(Env *env) {
  // TODO: INSERT YOUR PLANNER HERE
  State start_state = env->get_initial_conditions();
  State goal_state = env->get_goal_conditions();

  Tree tree(env, start_state, goal_state);
  // tree.addNode(start_state, start_node);
  OpenList open_list;
  open_list.insert(tree.getNodeData(start_state));
  // Blocks World example (CHANGE THIS)
  // cout << endl << "CREATING DEFAULT PLAN" << endl;
  while (!open_list.empty()) {
    NodePtr expanded_node = *open_list.begin();
    open_list.erase(open_list.begin());

    if (areConditionsSatisfied(expanded_node->state, goal_state)) {
      list<GroundedAction> actions;
      actions = tree.constructPlan(expanded_node->state);
      return actions;
    }

    auto available_actions = generateValidActions(env, expanded_node->state);

    for (auto action : available_actions) {
      State neighbor = getNextState(env, expanded_node->state, action);

      // if (tentative_g_val < tree.getNodeCost(expanded_node->state)) {
      //   tree.setParent(neighbor, expanded_node->state, action);
      // }

      // if (!tree.doesNodeExist(neighbor)) {
      //   NodePtr neighbor_node = std::make_shared<Node>();
      //   neighbor_node->state = neighbor;
      //   neighbor_node->g_val = expanded_node->g_val + 1;
      //   neighbor_node->f_val = heuristicFunction(neighbor, goal_state);
      //   tree.addNode(neighbor, neighbor_node);

      //   open_list.insert(neighbor_node);
      // } else {
      //   long int tentative_g_val = expanded_node->g_val + 1;
      //   if (tentative_g_val <)

      // }

      long int tentative_g_val = expanded_node->g_val + 1;

      if (tentative_g_val < tree.getNodeData(neighbor)->g_val) {
        tree.setParent(neighbor, expanded_node->state, action);
        // tree.setCost(neighbor, tentative_g_val);

        auto neighbor_itr = open_list.find(tree.getNodeData(neighbor));
        if (neighbor_itr != open_list.end()) {
          open_list.erase(neighbor_itr);
        }

        open_list.insert(tree.getNodeData(neighbor));
      }
    }
  }

  // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
  // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
  // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

  return list<GroundedAction>();
}

// clang-format off

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}