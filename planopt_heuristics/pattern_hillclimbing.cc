#include "pattern_hillclimbing.h"

#include "canonical_pdbs.h"

#include "../globals.h"

#include "../utils/logging.h"

using namespace std;

namespace planopt_heuristics
{
vector<set<int>> compute_causally_relevant_variables(const TNFTask &task)
{
  /*
        v is causally relevant for w
      iff
        v is a predecessor of w in the causal graph, or
        v is a successor of w in the causal graph and mentioned in the goal
      Since we are considering TNF tasks where all variables are mentioned in
      the goal, this condition simplifies to

        v is causally relevant for w
      iff
        v and w are neighbors in the causal graph.
      iff
        v and w occur in the same operator (and at least one of them changes the state).
    */

  int num_variables = task.variable_domains.size();
  vector<set<int>> relevant(num_variables);
  for (const TNFOperator &op : task.operators)
  {
    for (const TNFOperatorEntry &e1 : op.entries)
    {
      for (const TNFOperatorEntry &e2 : op.entries)
      {
        if (e1.variable_id != e2.variable_id &&
            (e1.precondition_value != e1.effect_value ||
             e2.precondition_value != e2.effect_value))
        {
          relevant[e1.variable_id].insert(e2.variable_id);
        }
      }
    }
  }
  return relevant;
}

bool HillClimber::fits_size_bound(const std::vector<Pattern> &collection) const
{
  /*
      Compute the number of abstract states in the given collection without
      explicitly computing the projections. Return true if the total size
      is below size_bound and false otherwise.
    */
  // exercicio (f)
  // calcular a soma dos estados abstratos: tem um total do conjunto, e um numero de estados pra cada pattern no conjunto
  int total = 0;
  for (const auto p : collection)
  { // pra cada pattern na coleção
    int val = 1;
    for (const auto e : p)
    {                                  // pra cada pattern dentro da coleção
      val *= task.variable_domains[e]; // multiplica pelo dominio da variável no pattern
    }
    total += val;           // soma no total do conjunto
    if (total > size_bound) // se passou do bound retorna falso
      return false;
  }
    cout<<"FIIIIIIIIIIIIIIITS"<<endl;
  return true;
}

HillClimber::HillClimber(const TNFTask &task, int size_bound, vector<TNFState> &&samples)
    : task(task),
      size_bound(size_bound),
      samples(move(samples)),
      causally_relevant_variables(compute_causally_relevant_variables(task))
{
}

vector<Pattern> HillClimber::compute_initial_collection()
{
  /*
      In TNF, every variable occurs in the goal state, so we create the
      collection {{v} | v \in V}.
    */

  vector<Pattern> collection;
  // exercício (f)
  for (const auto v : task.goal_state)
  { // pra cada variável citada no goal eu ponho na coleção
    Pattern p;
    p.push_back(v);
    collection.push_back(p);
  }
  cout<<task.goal_state<<endl;
  cout<<collection<<endl;
  cout<<"COMPUTEINITIAALLLLLL"<<endl;

  return collection;
}

vector<vector<Pattern>> HillClimber::compute_neighbors(const vector<Pattern> &collection)
{
  /*
      for each pattern P in the collection C:
          compute the set of variables that are causally relevant for any V in P
             (Use the precomputed information in causally_relevant_variables.)
          remove all variables from this set that already occur in P
          for each variable V in the resulting set:
              add the collection C' := C u {P u {V}} to neighbors

    */
  vector<vector<Pattern>> neighbors;

  // exercício (f)

  // pra cada P na coleção C:
  for(const auto p: collection){
    // computa o conjunto de variaveis casualmente relevantes pra qualquer variavel no patter n
    set<int> causally_relevant;
    for(const auto v:p){ // causally_relevant tem todas variaveis q sao casualmente relevantes pra qualquer variavel no pattern
      for( auto x : causally_relevant_variables[v] ) {
        causally_relevant.insert(x);
      }
    }

    for(const auto v : p){ // tira do set todas as variáveis que já ocorrem em p
      if(causally_relevant.find(v) != causally_relevant.end()){
        causally_relevant.erase(v);
      }
    }
    // pra cada variavel v no conjunto resultante
    for(const auto v: causally_relevant){
      vector<Pattern> clinha = collection;
      clinha.push_back(p);
      Pattern conj_v;
      conj_v.push_back(v);
      clinha.push_back(conj_v);
      neighbors.push_back(clinha);
    }
  }
  cout<<"NEIGHBOOOOOOOORSSS"<<endl;
  return neighbors;
}

vector<int> HillClimber::compute_sample_heuristics(const vector<Pattern> &collection)
{
  CanonicalPatternDatabases cpdbs(task, collection);
  vector<int> values;
  values.reserve(samples.size());
  for (const TNFState &sample : samples)
  {
    values.push_back(cpdbs.compute_heuristic(sample));
  }
  return values;
}

vector<Pattern> HillClimber::run()
{
  vector<Pattern> current_collection = compute_initial_collection();
  vector<int> current_sample_values = compute_sample_heuristics(current_collection);

  /*
      current := an initial candidate
      loop forever:
          next := a neighbor of current with maximal improvement over current
          if improvement(next) = 0:
              return current
          current := next

      To measure improvement, use compute_sample_heuristics to compute
      heuristic values for all sample state. Compare the result to
      current_sample_values and count the number of states that have a higher
      heuristic value. Remember to update current_sample_values when you
      modify current_collection.
    */

  // exercício (f)
  vector<Pattern> current = current_collection;
  while(true){
    auto neighbors = compute_neighbors(current);
    int next_current = -1;
    
    vector<Pattern> next_current_c; // o vizinho com maior improvment
    vector<int> next_current_h; // o valor da heuristica pro vizinho
    
    for(const auto n : neighbors){ // pra cada vizinho
      vector<int> neighbor_h = compute_sample_heuristics(n); // calcula a heuristica dele 
      int imp = 0;  // improvement é zero no inicio
      for(unsigned int i=0; i<neighbor_h.size(); i++){
        if(neighbor_h[i] > current_sample_values[i]){
          imp ++;
        } // conta em quantos estados melhorou
      }
      if(imp > next_current){ // se melhorou mais que o mais alto encontrado (começa em -1)
        next_current_c = n; // é o proximo corrente
        next_current_h = neighbor_h; // salva a heuristica pra ser o sample
        next_current = imp; // atualiza o maior vizinho
      }
    }
    if(next_current == -1){ // se não encontrou um vizinho melhor o melhor é o atual
      return current;
    }else{
      cout<<"AAIUDHIUASHDUi"<<endl;
      current = next_current_c; // se encontrou ele é o corrente
      current_sample_values = next_current_h;
    }   
  }
}
} // namespace planopt_heuristics
