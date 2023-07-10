/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: David Portugal (2011-2014), and Luca Iocchi (2014)
* and Mario Gianni (2016), and Luigi Freda (2016-Present) 
*********************************************************************/

typedef unsigned int uint;


typedef struct {
  int id, dist, elem_path;
  int path[1000];
  bool visit;
}s_path;

typedef struct {
  int id, elem_path;
  double dist;
  int path[1000];
  bool visit;
}s_path_mcost;

typedef struct {
  uint current_vertex, next_vertex;
  double time_then;
  uint num_possible_neighs;
  uint id_neighbors [8];
  double idleness_old[8];
  uint node_count [8];
  double entropy;

}reinforcement_learning;


//inline long double log2(const long double x)
//{
//    return  log(x) * M_LOG2E;
//}

uint random (uint current_vertex, Vertex *vertex_web);

uint conscientious_reactive (uint current_vertex, Vertex *vertex_web, double *instantaneous_idleness);

uint heuristic_conscientious_reactive (uint current_vertex, Vertex *vertex_web, double *instantaneous_idleness);

uint greedy_bayesian_strategy (uint current_vertex, Vertex *vertex_web, double *instantaneous_idleness, double G1, double G2, double edge_min);

int count_intention (uint vertex, int *tab_intention, int nr_robots);

uint state_exchange_bayesian_strategy (uint current_vertex, Vertex *vertex_web, double *instantaneous_idleness, int *tab_intention, int nr_robots, double G1, double G2, double edge_min);

void dijkstra( uint source, uint destination, int *shortest_path, uint &elem_s_path, Vertex *vertex_web, uint dimension);

int is_neigh(uint vertex1, uint vertex2, Vertex *vertex_web, uint dimension);

void dijkstra_mcost( uint source, uint destination, int *shortest_path, uint &elem_s_path, Vertex *vertex_web, double new_costs[][8], uint dimension);

uint heuristic_pathfinder_conscientious_cognitive (uint current_vertex, Vertex *vertex_web, double *instantaneous_idleness, uint dimension, uint *path);

//Check if an element belongs to a table
bool pertence (int elemento, int *tab, int tam_tab);

int pertence_idx (int elemento, int *tab, int tam_tab);

bool UHC (Vertex *vertex_web, int v1, int *caminho_principal, uint dimension);

void clear_visited (Vertex *vertex_web, uint dimension);

bool procurar_ciclo (Vertex *vertex_web, uint dimension, int *caminho_principal, int &elem_caminho, int &custo_max, int seed);

int devolve_viz_unicos (Vertex *vertex_web, int vertice);

int longest_path (Vertex *vertex_web, int origem, int destino, int *lista_v1v, int i_list, int dimension, int seed, int *caminho_parcial, int &elem_cp);

bool caminho_apartir_vizinhos_unicos (Vertex *vertex_web, int dimension, int *caminho_principal, int &elem_max, int &custo_max, int seed);

bool verificar_arco_cp (int no_1, int no_2, int *caminho_principal, int elem_cp);

bool check_visited (Vertex *vertex_web, int dimension);

int devolve_vizinhos_nao_visitados (Vertex *vertex_web, int dimension, int vertice);

bool computar_caminho_de_ida (Vertex *vertex_web, int dimension, int *caminho_de_ida, int &elem_c_ida, int *caminho_principal, int elem_cp, int num_arcos);

int computar_custo_caminho_final (Vertex *vertex_web, int *caminho_final, int elem_caminho_final);

int cyclic (uint dimension, Vertex *vertex_web, int *caminho_final);

void shift_cyclic_path (uint start_vertex, int *caminho_final, int elem_caminho_final);

uint get_MSP_dimension (const char* msp_file);

void get_MSP_route (uint *route, uint dimension, const char* msp_file);

void create_source_and_dest_tables(Vertex *vertex_web, uint *source, uint *destination, uint dimension);

void get_hist_sort(Vertex *vertex_web, double *hist_sort, uint dimension);

int get_hist_idx_from_edge_cost (double *hist_sort, uint size, double edge_cost);

int get_hist_idx (uint *source, uint *destination, uint source_vertex, uint dest_vertex, uint hist_dimension);

double get_edge_cost_between (Vertex *vertex_web, uint vertex_A, uint vertex_B);

void load_real_histogram(double *real_histogram, uint size_hist, char* filename);

void normalize_histogram(double *real_histogram, double *histogram, uint size_hist);

int pertence_uint_idx (uint elemento, uint *tab, uint tam_tab);

int get_min(uint *tab, uint tam_tab);

double get_min_dbl(double *tab, uint tam_tab);

int get_max(uint *tab, uint tam_tab);

double get_max_dbl(double *tab, uint tam_tab);

void write_histogram_to_file (Vertex *vertex_web, double *real_histogram, double *histogram, uint *source, uint *destination, uint hist_dimension, uint number, uint robotid);

void write_reward_evolution(double reward, uint robotid);

void update_likelihood_old (reinforcement_learning RL, double *real_histogram, double *hist_sort, uint size_hist, Vertex *vertex_web);

void update_likelihood (reinforcement_learning RL, double *real_histogram, uint *source, uint *destination, uint hist_dimension, Vertex *vertex_web, int minimum_global_node_count, uint robotid);

void update_likelihood_new (reinforcement_learning RL, uint *node_count_table, double *inst_idleness, uint dimension, double *real_histogram, uint *source, uint *destination, uint hist_dimension, Vertex *vertex_web, uint robotid);

int learning_algorithm(uint current_vertex, Vertex *vertex_web, double *instantaneous_idleness, double *avg_idleness, int *tab_intention, double *histogram, uint *source, uint *destination, uint hist_dimension, int nr_robots, int id_robot, uint *node_count, reinforcement_learning &RL);


 