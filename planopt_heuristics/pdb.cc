#include "pdb.h"

#include "../utils/logging.h"

#include <queue>

using namespace std;

namespace planopt_heuristics {

/*
  An entry in the queue is a tuple (h, i) where h is the goal distance of state i.
  See comments below for details.
*/
using QueueEntry = pair<int, int>;

PatternDatabase::PatternDatabase(const TNFTask &task, const Pattern &pattern)
    : projection(task, pattern) {
    /*
      We want to compute goal distances for all abstract states in the
      projected task. To do so, we start by assuming every abstract state has
      an infinite goal distance and then do a backwards uniform cost search
      updating the goal distances of all encountered states.

      Instead of searching on the actual states, we use perfect hashing to
      run the search on the hash indices of states. To go from a state s to its
      index use rank(s) and to go from an index i to its state use unrank(i).
    */
    const TNFTask &projected_task = projection.get_projected_task();
    distances.resize(projected_task.get_num_states(), numeric_limits<int>::max());

    /*
      Priority queues usually order entries so the largest entry is the first.
      By using the comparator greater<T> instead of the default less<T>, we
      change the ordering to sort the smallest element first.
    */
    priority_queue<QueueEntry, vector<QueueEntry>, greater<QueueEntry>> queue;
    /*
      Note that we start with the goal state to turn the search into a regression.
      We also have to switch the role of precondition and effect in operators
      later on. This is sufficient to turn the search into a regression since
      the task is in TNF.
    */
    queue.push({0, projection.rank_state(projected_task.goal_state)});
    
    // exercício (b)

    while(!queue.empty()){
      int atual = queue.top().second;
      int custo_atual = queue.top().first;
      queue.pop();
      auto state_atual = projection.unrank_state(atual);
        
      for(const auto o : projected_task.operators){
        // pra cada operador
        bool applicable = true;
        for(const auto e : o.entries){
          if(state_atual[e.variable_id] != e.effect_value) // se caiu aqui não pode aplicar
          {
            applicable = false;
            break;
          }
        }
        if(applicable) // é aplicavel
        {
          int custo = custo_atual + o.cost; // o custo é o atual + o desse operador
          TNFState novo_estado = state_atual; // crio um novo estado(pra por na fila de novo)
          for(const auto e : o.entries){
            novo_estado[e.variable_id] = e.precondition_value; // aplico o operador (as pré condições)
          } 
          int ranked_novo_estado = projection.rank_state(novo_estado); 
          if( custo < distances[ranked_novo_estado]){
            distances[ranked_novo_estado]=custo;
            queue.push({custo, ranked_novo_estado});
          }
        }
      }
    }

    
}

int PatternDatabase::lookup_distance(const TNFState &original_state) const {
    TNFState abstract_state = projection.project_state(original_state);
    int index = projection.rank_state(abstract_state);
    return distances[index];

}

}
