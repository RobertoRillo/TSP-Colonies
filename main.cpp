#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <vector>
using namespace std;

const int INF = numeric_limits<int>::max();

class Graph {
private:
  int N;
  vector<vector<int>> adjacencyMatrix;
  vector<vector<int>> capacityMatrix;
  int find(vector<int> &parent, int i) {
    while (parent[i] != i)
      i = parent[i];
    return i;
  }

  void unionSet(vector<int> &parent, int x, int y) {
    int xSet = find(parent, x);
    int ySet = find(parent, y);
    parent[ySet] = xSet;
  }

public:
  Graph(int _N) : N(_N) {
    adjacencyMatrix = vector<vector<int>>(N, vector<int>(N, 0));
    capacityMatrix = vector<vector<int>>(N, vector<int>(N, 0));
  }
  int getN() const { return N; }
  void addEdge(int u, int v, int weight, int capacity) {
    adjacencyMatrix[u][v] = weight;
    adjacencyMatrix[v][u] = weight;
    capacityMatrix[u][v] = capacity;
  }

  void addCapacity(int u, int v, int capacity) {
    capacityMatrix[u][v] = capacity;
  }


  int getPathWeight(const vector<int> &path) {
    int weight = 0;
    for (int i = 0; i < N - 1; ++i) {
      weight += adjacencyMatrix[path[i]][path[i + 1]];
    }
    weight += adjacencyMatrix[path[N - 1]][path[0]]; // Volver al inicio
    return weight;
  }
  vector<int> findShortestTSPNearestNeighbor() {
    vector<int> path;
    vector<bool> visited(N, false);

    int current = 0; // nodo actual

    path.push_back(current);
    visited[current] = true;

    for (int i = 1; i < N; ++i) {
      int next = findNearestNeighbor(current, visited);
      path.push_back(next);
      visited[next] = true;
      current = next;
    }

    return path;
  }

  int findNearestNeighbor(int current, vector<bool> &visited) {
    int nearestNeighbor = -1;
    int minDistance = INF;

    for (int v = 0; v < N; ++v) {
      if (!visited[v] && adjacencyMatrix[current][v] < minDistance) {
        nearestNeighbor = v;
        minDistance = adjacencyMatrix[current][v];
      }
    }

    return nearestNeighbor;
  }

  int maxFlow(int source, int sink) {
    int maxFlow = 0;

    while (true) {
      vector<int> parent(N, -1);
      queue<int> q;
      q.push(source);
      parent[source] = source;

      while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < N; v++) {
          if (parent[v] == -1 && capacityMatrix[u][v] > 0) {
            parent[v] = u;
            q.push(v);
          }
        }
      }

      if (parent[sink] == -1) {
        break;
      }

      int pathFlow = numeric_limits<int>::max();

      for (int v = sink; v != source; v = parent[v]) {
        int u = parent[v];
        pathFlow = min(pathFlow, capacityMatrix[u][v]);
      }

      maxFlow += pathFlow;

      for (int v = sink; v != source; v = parent[v]) {
        int u = parent[v];
        capacityMatrix[u][v] -= pathFlow;
        capacityMatrix[v][u] += pathFlow;
      }
    }

    return maxFlow;
  }
  vector<pair<int, int>> kruskalMST() {
    vector<pair<int, int>> edges;

    // Crear un vector de aristas
    for (int u = 0; u < N; ++u) {
      for (int v = u + 1; v < N; ++v) {
        if (adjacencyMatrix[u][v] != 0) {
          edges.push_back({u, v});
        }
      }
    }

   
    sort(edges.begin(), edges.end(),
         [this](pair<int, int> &a, pair<int, int> &b) {
           return adjacencyMatrix[a.first][a.second] <
                  adjacencyMatrix[b.first][b.second];
         });

    vector<pair<int, int>> result;
    vector<int> parent(N);
    iota(parent.begin(), parent.end(), 0); // Inicializar los padres

    int edgeCount = 0;
    for (const auto &edge : edges) {
      int u = edge.first;
      int v = edge.second;
      int uParent = find(parent, u);
      int vParent = find(parent, v);

      if (uParent != vParent) {
        result.push_back({u, v});
        unionSet(parent, uParent, vParent);
        edgeCount++;
      }

      if (edgeCount == N - 1) // El MST tiene N-1 aristas
        break;
    }

    return result;
  }
};
struct Chromosome {
  vector<int> path;
  int fitness;

  Chromosome(int size) : fitness(0) { path.resize(size); }

  void evaluateFitness(Graph &graph) { fitness = graph.getPathWeight(path); }

  void mutate() {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dis(0, path.size() - 1);

    int idx1 = dis(gen);
    int idx2 = dis(gen);

    swap(path[idx1], path[idx2]);
  }
};

Chromosome tournamentSelection(vector<Chromosome> &population,
                               int tournamentSize) {
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<int> dis(0, population.size() - 1);

  Chromosome best = population[dis(gen)];

  for (int i = 1; i < tournamentSize; ++i) {
    Chromosome contender = population[dis(gen)];
    if (contender.fitness < best.fitness) {
      best = contender;
    }
  }

  return best;
}

Chromosome crossover(const Chromosome &parent1, const Chromosome &parent2) {
  Chromosome offspring(parent1.path.size());

  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<int> dis(0, parent1.path.size() - 2);

  int startPos = dis(gen);
  int endPos = dis(gen);
  if (endPos < startPos) {
    swap(startPos, endPos);
  }

  for (int i = startPos; i <= endPos; ++i) {
    offspring.path[i] = parent1.path[i];
  }

  int k = 0;
  for (int i = 0; i < parent2.path.size(); ++i) {
    if (k == startPos) {
      k = endPos + 1;
    }
    if (find(offspring.path.begin(), offspring.path.end(), parent2.path[i]) ==
        offspring.path.end()) {
      offspring.path[k++] = parent2.path[i];
    }
  }

  return offspring;
}

void geneticAlgorithm(Graph &graph) {
  const int POPULATION_SIZE = 50;
  const int TOURNAMENT_SIZE = 5;
  const double MUTATION_RATE = 0.1;
  const int MAX_GENERATIONS = 100;

  vector<Chromosome> population;
  population.reserve(POPULATION_SIZE);

  for (int i = 0; i < POPULATION_SIZE; ++i) {
    Chromosome chromosome(graph.getN());
    iota(chromosome.path.begin(), chromosome.path.end(), 0);
    random_shuffle(chromosome.path.begin() + 1, chromosome.path.end());
    chromosome.evaluateFitness(graph);
    population.push_back(chromosome);
  }

  for (int generation = 0; generation < MAX_GENERATIONS; ++generation) {
    vector<Chromosome> newPopulation;

    for (int i = 0; i < POPULATION_SIZE; ++i) {
      Chromosome parent1 = tournamentSelection(population, TOURNAMENT_SIZE);
      Chromosome parent2 = tournamentSelection(population, TOURNAMENT_SIZE);

      Chromosome offspring = crossover(parent1, parent2);

      if ((static_cast<double>(rand()) / RAND_MAX) < MUTATION_RATE) {
        offspring.mutate();
      }

      offspring.evaluateFitness(graph);
      newPopulation.push_back(offspring);
    }

    population = newPopulation;
  }

  Chromosome bestSolution =
      *min_element(population.begin(), population.end(),
                   [](const Chromosome &a, const Chromosome &b) {
                     return a.fitness < b.fitness;
                   });

  
  cout << "La mejor ruta encontrada es: ";
  for (int i = 0; i < bestSolution.path.size(); ++i) {
    char colony = 'A' + bestSolution.path[i];
    cout << colony << " ";
  }
  char colony = 'A' + bestSolution.path[0];
  cout << colony << endl;

  cout << "La longitud de la mejor ruta es: " << bestSolution.fitness << endl;
}

int main() {
  ifstream input("datos.txt");
  int N;
  input >> N;

  Graph graph(N);

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      int weight;
      input >> weight;
      graph.addEdge(i, j, weight, 0);
    }
  }

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      int capacity;
      input >> capacity;
      graph.addCapacity(i, j, capacity);
    }
  }

  input.close();

  int start = 0;
  int end = 3;

  vector<pair<int, int>> minimumSpanningTree = graph.kruskalMST();
  cout << "Conexiones mínimas entre colonias:\n";
  for (const auto &edge : minimumSpanningTree) {
    char colony1 = 'A' + edge.first;
    char colony2 = 'A' + edge.second;
    cout << colony1 << "->" << colony2 << endl;
  }
   cout << endl;
  
  vector<int> shortestTSPPath = graph.findShortestTSPNearestNeighbor();
  cout << "La ruta a seguir por el personal que reparte correspondencia es:"
       << endl;
  for (int i = 0; i < shortestTSPPath.size(); i++) {
    char colony = 'A' + shortestTSPPath[i];
    cout << colony << " ";
  }
  char colony = 'A' + shortestTSPPath[0];
  cout << colony << endl;
  cout << endl;
  
  cout << "Implementación Original " << endl;
  geneticAlgorithm(graph);
  cout << endl;

  int maxFlowValue = graph.maxFlow(start, end);
  cout << "El valor de flujo máximo de información del nodo inicial al nodo "
          "final es: "
       << maxFlowValue << " unidades" << endl;

  return 0;
}
