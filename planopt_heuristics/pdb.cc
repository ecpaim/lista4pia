#include "pdb.h"

#include "../utils/logging.h"

#include <queue>
#include <set>
using namespace std;

namespace planopt_heuristics
{

/*
  An entry in the queue is a tuple (h, i) where h is the goal distance of state i.
  See comments below for details.
*/
using QueueEntry = pair<int, int>;

PatternDatabase::PatternDatabase(const TNFTask &task, const Pattern &pattern)
    : projection(task, pattern)
{
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

  while (!queue.empty())
  {
    int current_distance = queue.top().first;
    int state = queue.top().second;
    queue.pop();

    if (distances[state] > current_distance) // se vai atualizar a distances pra estado 
    {
      distances[state] = current_distance;
      TNFState state_unranked = projection.unrank_state(state);
      for (const auto op : projected_task.operators) // pra cada operador
      {
        bool applicable = true;
        TNFState new_state(state_unranked); // o estado é uma cópia do estado antigo
        for (const auto v : op.entries)
        {
          if (state_unranked[v.variable_id] != v.effect_value) // se não foi aplicado o efeito tem valor diferente do valor do estado
          {
            applicable = false;
            break;
          }
        }
        if (applicable) // se o operador for completamente aplicavel eu adiciono na fila
        {
          for(const auto v : op.entries){
            new_state[v.variable_id] = v.precondition_value; // aplico a pré condição
          }
          queue.push({current_distance + op.cost, projection.rank_state(new_state)});
        }
      }
    }
  }
}
 // namespace planopt_heuristics

int PatternDatabase::lookup_distance(const TNFState &original_state) const
{
  TNFState abstract_state = projection.project_state(original_state);
  int index = projection.rank_state(abstract_state);
  return distances[index];
}
}
