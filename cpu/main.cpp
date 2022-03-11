#include <mutex>
#include <math.h>
#include <chrono>
#include <random>
#include <vector>
#include <mutex>
 #include <iostream>
#include <algorithm>
using namespace std;

const int BIT_SIZE = 32;          // size of node ID's
const int DEMAND_BITS = 8;        // bits allocated towards the demand from the node ID
int MAX_GENERATIONS = 2;
float CROSSOVER_RATE = 0.60;
float MUTATION_RATE = 0.30;

int capacity = 150;
int N = 10;
int C = 4;
int M = (1 + ((24 * N - 1) / C)) * C;
int data[10][3] = {{365, 689, 0}, {146, 180, 38}, {792, 5, 51}, {658, 510, 73}, {461, 270, 70}, {299, 531, 58}, {812, 228, 54}, {643, 90, 1}, {615, 630, 98}, {258, 42, 62}};

// shuffle a vector
void shuffle(vector<int> &vect , int start, int end) {
    for (int i = start; i < end; i++) {     // for each node
        int r = 4 + (rand() % (end - 5));   // get the random position
        swap(vect[i], vect[r]);             // swap the current node with a random node further along
    }
}

// encrypting function for transforming x, y and demand into a single unique ID
int encrypt(int x, int y, int demand) {
    int bitShift = pow(2, (BIT_SIZE - DEMAND_BITS)/2);      // bits to shift
    int location = x * bitShift + y;                        // pair the x and y
    return demand * bitShift * bitShift + location;         // return the unique ID for the node by pairing the x, y and demand
}

// get the x, y, or demand from the unique ID
vector<int> decrypt(int ID) {
    int bitShift = pow(2, (BIT_SIZE - DEMAND_BITS)/2);
    int x =  (ID % (bitShift * bitShift)) / bitShift;
    int y = ID % bitShift;
    int demand = ID/(bitShift * bitShift);
    return vector<int> {x, y, demand};
}

// initialize a random phenotype: for each solution in the population, add a randomized array of the nodes and zeros
vector<int> init_population(int depot, vector<int> &nodes) {
    vector<int> pheno;
    pheno.assign(N + 1, depot);                                             // fill the phenotype with the depot's ID
    for (int n = 0; n < N; n++) {pheno.insert(pheno.end() - 5, nodes[n]);}  // add in the rest of the nodes to the phenos (1 place over from the end of the vector)
    shuffle(pheno, 3, pheno.size() - 1);                                    // shuffle the phenotype
    pheno[1] = 0;                                                           // assign the generation to the pheno
    return pheno;
}

// get the cost of the solution
float cost(vector<int> &pheno, int start, int end) {
    float distance = 0;
    vector<int> prev, curr = decrypt(pheno[start - 1]);

    // iterate through the nodes
    for (int node = start; node <= end; node++) {

        // get the location of the previous and current node
        prev = curr;
        curr = decrypt(pheno[node]);

        // get the euclidean distance from the previous node and current node and add it to the total
        distance += sqrt(pow(prev[0] - curr[0], 2) + pow(prev[1] - curr[1], 2));

    }
    return distance;
}

// establish the routes based of capacity (fit as many nodes into a route)
void routes(vector<int> &pheno, int depot, int capacity) {
    int bitShift = pow(2, BIT_SIZE - DEMAND_BITS);   // the bits to shift the ID over to get the demand
    int load = 0;                                   // the demand of a route

    // for each gene in the pheno (skipping the first two for fitness and generation, and an additional one to establish the first route and the last one to finish the last route)
    for (int gene = 3; gene < pheno.size() - 1; gene++) {
        if (pheno[gene] == depot) {                              // if the current gene is a depot
            pheno.erase(pheno.begin() + gene);                   // then it is popped out of the vector
            gene -= 1;                                           // the iterator is decreased one to not skip the next element
        } else {                                                 // else the current gene is not a depot
            int demand = pheno[gene]/bitShift;                   // demand of current node is found
            if (load + demand <= capacity) {load += demand;}     // demand is added to load if capacity is not violated
            else {                                               // else adding demand violates the capacity
                pheno.insert(pheno.begin() + gene, depot);       // a depot is inserted to complete a route
                load = 0;                                        // the load is reset
            }
        }
    }

    // add depots to pad the pheno
    for (int i = pheno.size(); i < 2*N + 1; i++) {pheno.push_back(depot);}
}

// conduct two opt optimization
void two_opt(vector<int> &pheno, int depot) {
    for (int gene = 3; gene < pheno.size() - 1;) {  // iterate through the genes
        if (pheno[gene] == depot) {gene++;}        // if the current gene is a depot, then pass over it
        else {                                     // else the current gene is not a depot

            // find the number of nodes in the route
            int nodesInRoute = 1;
            while (pheno[gene + nodesInRoute] != depot && gene + nodesInRoute != pheno.size()) {nodesInRoute += 1;}

            // declare the start and end of the route
            int start = gene, end = gene + nodesInRoute;

            // two opt: continue the loop until no improvement is made
            bool improved = true;
            while (improved) {

                // set the imrpovement flag to false
                improved = false;

                // find the distance of the route
                float best = cost(pheno, start, end);

                // iterate through each possible swap
                REPEAT:
                for (int i = start; i < end - 1; i++) {
                    for (int j = i + 1; j < end; j++) {

                        // perform the swap and get the distance of the new route
                        iter_swap(pheno.begin() + i, pheno.begin() + j);
                        float distance = cost(pheno, start, end);

                        // if the new distance improves then the new best distance is recorded
                        if (distance < best) {
                            best = distance;
                            improved = true;
                            goto REPEAT;

                        // else the swap is undone
                        } else {iter_swap(pheno.begin() + j, pheno.begin() + i);}
                    }
                }
            }

            // after the two-opt, pass over the route
            gene += nodesInRoute + 1;
        }
    }
}

// 2-point crossover
void crossover(vector<int> &parent1, vector<int> &parent2, vector<int> &offspring1, vector<int> &offspring2, int generations) {

    // get randomize locations of the cuts
    int cut1 = rand() % (2*N - 7) + 4;
    int cut2 = rand() % (2*N - 3 - cut1) + cut1 + 2;
    
    // add the first part of each parent to the respective child (upto cut 1)
    copy(parent1.begin(), parent1.begin() + cut1, back_inserter(offspring1));
    copy(parent2.begin(), parent2.begin() + cut1, back_inserter(offspring2));

    // add the second part of each parent to the respective child (between cut 1 and cut 2)
    copy(parent2.begin() + cut1, parent2.begin() + cut2, back_inserter(offspring1));
    copy(parent1.begin() + cut1, parent1.begin() + cut2, back_inserter(offspring2));

    // add the last part of each parent to the respective child (after cut 2)
    copy(parent1.begin() + cut2, parent1.end(), back_inserter(offspring1));
    copy(parent2.begin() + cut2, parent2.end(), back_inserter(offspring2));

    //input the generation it is from
    offspring1[1] = generations;
    offspring2[1] = generations;
}

// perform inverse mutation
void mutation(vector<int> &pheno) {
    int cut1 = rand() % (2*N - 4) + 3;                  // location of the first cut (between point [start + 3] to [end - 2])
    int cut2 = rand() % (2*N - cut1 - 1) + 1;           // loction of second cut (between [cut1 + 1] to [end - 2])
    reverse(pheno.begin() + cut1, pheno.end() - cut2);  // reverse the nodes between cut1 and cut2
}

// replace duplicate nodes with missing nodes
void adjust(vector<int> &pheno, vector<int> &nodes, int depot) {
    vector<int> missing;

    // find missing nodes
    for (int i = 1; i < N; i++) {
        bool found = false;

        // iterate through the nodes and pheno to find if nodes are missing
        for (int j = 0; j < pheno.size(); j++) {
            if (nodes[i] == pheno[j]) {
                found = true;
                break;
            }
        }

        // if the node was not found then it is added to the missing vector
        if (found == false) {missing.push_back(nodes[i]);}
    }

    // replace duplicates with missing nodes
    for (int i = 3; i < pheno.size() - 1; i++) {

        // continue if the current node is not a depot
        if (pheno[i] != depot) {

            // iterate through the remaining nodes
            for (int j = i + 1; j < pheno.size() - 1; j++) {

                // if the current node is a duplicate replace it with a missing node
                if (pheno[i] == pheno[j]) {
                    for (int m = 0; m < M; m++) {pheno[0] = cost(pheno, 3, 2*N);}

                    // if there are still non depot nodes replace it with one of them
                    if (missing.size() > 0) {
                        pheno[j] = missing[0];
                        missing.erase(missing.begin());
                    }

                    // else replace it with a depot
                    else {pheno[j] = depot;}
                }
            }
        }
    }

    // replace depots with remaining missing nodes
    int last = 3;
    for (int i = 0; i < missing.size(); i++) {
        for (int j = last; j < pheno.size(); j++) { // iterate through the nodes, starting from the last replacement made
            if (pheno[j] == depot) {                // if the current node is a depot
                pheno[j] = missing[0];              // then replace the depot node with a missing node
                missing.erase(missing.begin());     // delete it from the missing vector
                last = j;                           // update the last location the depot was found
                break;                              // break out of the loop
            }
        }
    }
}

int main() {
    // seed for randomization
    srand(time(0));

    // transform x, y, and demand into single unique number (ID)
    vector<int> nodes;
    for (int n = 0; n < N; n++) {nodes.push_back(encrypt(data[n][0], data[n][1], data[n][2]));}

    // ID of the depot
    int depot = nodes[0];

    // initialize population (and offspring matrix)
    vector<vector<int>> population;
    for (int m = 0; m < M; m++) {population.push_back(init_population(depot, nodes));}

    // establish routes
    for (int m = 0; m < M; m++) {routes(population[m], depot, capacity);}
cout << "257" << endl;
    // two-opt optimization
    for (int m = 0; m < M; m++) {two_opt(population[m], depot);}
cout << "260" << endl;
    // get the fitness value for each phenotype
    for (int m = 0; m < M; m++) {population[m][0] = cost(population[m], 3, 2*N);}
cout << "263" << endl;
    // create an array for randomly pairing parents
    vector<int> randVect;
    for (int m = 0; m < M; m++) {randVect.push_back(m);}
cout << "267" << endl;
    for (int generations = 1; generations < MAX_GENERATIONS; generations++) {
cout << "269" << endl;
        // shuffle the indices array for pairing parent chromosomes
        shuffle(randVect, 0, M);
cout << "272" << endl;
        // 2-Point Crossover
        vector<vector<int>> offspring;
        for (int m = 1; m < M; m ++) {

            // get a random number to see if it should be crossover
            float chance = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            // only perform the crossover for certain pairs
            if (chance <= CROSSOVER_RATE) {

                // declare the parent and offspring vectors
                vector<int> parent1 = population[randVect[m]];
                vector<int> parent2 = population[randVect[m + 1]];
                vector<int> offspring1, offspring2;

                // get the offspring
                crossover(parent1, parent2, offspring1, offspring2, generations);
                offspring.emplace_back(offspring1);
                offspring.emplace_back(offspring2);
            }
        }
cout << "294" << endl;
        // Inverse Mutation
        for (int m = 0; m < M; m++) {

            // get a random number to see if it should be crossover
            float chance = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            // only perform the crossover for certain pairs
            if (chance <= MUTATION_RATE) {mutation(offspring[m]);}
        }
cout << "304" << endl;
        // Adjust the Population
        for (int m = 0; m < M; m++) {adjust(offspring[m], nodes, depot);}

        // establish the routes for the new phenotypes
        for (int m = 0; m < M; m++) {routes(offspring[m], capacity, depot);}

        // perform two opt optimization on the offspring
        for (int m = 0; m < M; m++) {two_opt(offspring[m], depot);}

        // get the fitness value for each new phenotype
        for (int m = 0; m < M; m++) {offspring[m][0] = cost(offspring[m], 3, 2*N + 1);}

        // combine the population and offspring
        for (int m = 0; m < M; m++) {offspring.emplace_back(population[m]);}

        // get the order and fitness of the population and offspring
        vector<vector<int>> rankings;
        for (int m = 0; m < offspring.size(); m++) {rankings.push_back(vector<int> {m, offspring[m][0]});}

        // sort the population + offspring by fitness
        sort(rankings.begin(), rankings.end(), [](const vector<int> &a, const vector<int> &b) {return a[1] < b[1];});

        // select the best performing individuals for the next generation (the first m individuals since it is sorted)
        population.clear();
        for (int m = 0; m < M; m++) {population.push_back(offspring[m]);}

        for (int i = 0; i < M; i++) {
            // for (int j = 0; j < population[i].size(); j++) {
            //     cout << population[i][j] << " ";
            // }
            cout << population[i].size() << " " << i << " " << M << endl;
        }
    
    }
    return 0;
}
