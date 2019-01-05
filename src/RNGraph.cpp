#include "RNGraph.h"

RNAdyacencyList::RNAdyacencyList(){
    adyacencies = new std::list<RNGraphEdge*>();
}

RNAdyacencyList::~RNAdyacencyList(){
    clear();
    delete adyacencies;
}

void RNAdyacencyList::insert(RNGraphEdge* edge){
    if(edge){
        if(!contains(edge->getDestination())){
            adyacencies->emplace_back(edge);   
        }
    }
}

void RNAdyacencyList::removeEdge(int edgeName){
    std::list<RNGraphEdge*>::iterator it;
    bool deleted = false;
    for(it = adyacencies->begin(); it != adyacencies->end() and not deleted; it++){
        if((*it)->getDestination() == edgeName){
            deleted = true;
        }
    }
    adyacencies->erase(it);
}

bool RNAdyacencyList::contains(int edgeName){
    bool r = false;
    std::list<RNGraphEdge*>::iterator it;
    for(it = adyacencies->begin(); it != adyacencies->end() and not r; it++){
        if((*it)->getDestination() == edgeName){
            r = true;
        }
    }
    return r;
}

void RNAdyacencyList::clear(){
    adyacencies->erase(adyacencies->begin(), adyacencies->end());
}

bool RNAdyacencyList::empty() const{
    return (adyacencies->size() == 0);
}

int RNAdyacencyList::size() const{
    return adyacencies->size();
}

RNGraphEdge* RNAdyacencyList::at(int index){
    return *(std::next(adyacencies->begin(), index));
}

const std::string RNAdyacencyList::toString() const{
    std::ostringstream print_str;
    print_str.str("");
    print_str.clear();
    std::list<RNGraphEdge*>::iterator it;
    for(it = adyacencies->begin(); it != adyacencies->end(); it++){
        print_str << (*it)->getDestination() << " ";
    }
    return print_str.str();
}

RNGraph::RNGraph(){
	graph = new std::map<int, RNAdyacencyList*>();
}

RNGraph::~RNGraph(){
    clear();
    delete graph;
}
	
void RNGraph::addNode(const int& node){
    graph->emplace(node, new RNAdyacencyList());
}

int RNGraph::addEdge(const int& src, const int& dst, const float& weight){
    int res = RN_NONE;
    std::map<int, RNAdyacencyList*>::iterator itSrc;
    std::map<int, RNAdyacencyList*>::iterator itDst;
    itSrc = graph->find(src);
    itDst = graph->find(dst);

    if(itSrc != graph->end() and itDst != graph->end()){
        RNGraphEdge* edge = new RNGraphEdge(dst, weight);
        RNAdyacencyList* adys = NULL;
        adys = itSrc->second;
        adys->insert(edge);
        graph->at(src) = adys;
        res = RN_OK;
    }
    return res;
}

const std::string RNGraph::toString() const{
    std::ostringstream print_str;
    print_str.str("");
    print_str.clear();

    std::map<int, RNAdyacencyList*>::iterator it;
    for(it = graph->begin(); it != graph->end(); it++){
        print_str << it->first << " -> " <<  it->second->toString() << "\n";
    }
    return print_str.str();
}

void RNGraph::removeNode(int node){
    std::map<int, RNAdyacencyList*>::iterator it;
    RNAdyacencyList* adys = NULL;
    for(it = graph->begin(); it != graph->end(); it++){
        adys = it->second;
        adys->removeEdge(node);
        graph->at(it->first) = adys;
    }
    it = graph->find(node);
    graph->erase(it);
}

void RNGraph::removeEdge(int src, int dst){
    RNAdyacencyList* adys = NULL;
    std::map<int, RNAdyacencyList*>::iterator it;

    it = graph->find(src);
    if(it != graph->end()){
        adys = it->second;
        adys->removeEdge(dst);
        graph->at(src) = adys;
    }
}

RNAdyacencyList* RNGraph::getAdyacencies(int node) const{
    RNAdyacencyList* adys = NULL;
    std::map<int, RNAdyacencyList*>::iterator it;

    it = graph->find(node);
    if(it != graph->end()){
        adys = it->second;
    }
    return adys;
}

std::list<int> RNGraph::branchAndBound(const int& src, const int& dst) const{
    std::list<int> path;
    std::list<int> visited;
    branchAndBound(src, dst, &path, &visited);
    return path;
}

void RNGraph::branchAndBound(const int& src, const int& dst, std::list<int>* cerrados, std::list<int>* visitados) const{
    visitados->emplace_back(src);
    if(src == dst){
        cerrados->emplace_back(src);
        return;
    } else {
        RNAdyacencyList* ady = this->getAdyacencies(src);
        if(not ady->empty()){
            bool finalNodeFound = false;
            for(int i = 0; i < ady->size() and not finalNodeFound; i++){
                bool visitedNode = std::find(visitados->begin(), visitados->end(), ady->at(i)->getDestination()) != visitados->end();
                if(not visitedNode){
                    branchAndBound(ady->at(i)->getDestination(), dst, cerrados, visitados);
                }
                finalNodeFound = std::find(cerrados->begin(), cerrados->end(), dst) != cerrados->end();
            }

            finalNodeFound = std::find(cerrados->begin(), cerrados->end(), dst) != cerrados->end();
            if(finalNodeFound){
                cerrados->emplace_front(src);
            }
        } else {
            printf("ady list empty\n");
        }
    }
}

std::map<int, int> RNGraph::shortestPath(const int& src) const{
    std::map<int, float> distances;
    std::map<int, int> froms;
    std::priority_queue<std::pair<int, float>, std::vector<std::pair<int, float> >, std::greater<std::pair<int, float> > > pq;
    std::map<int, RNAdyacencyList*>::iterator graph_it;
    for(graph_it = graph->begin(); graph_it != graph->end(); graph_it++){
        distances.emplace(graph_it->first, std::numeric_limits<float>::infinity());
        froms.emplace(graph_it->first, RN_NONE);
    }
    pq.emplace(std::pair<int, float>(src, 0.0));
    distances[src] = 0.0;

    while(!pq.empty()){
        int current_node = pq.top().first;
        pq.pop();

        RNAdyacencyList* ady = this->getAdyacencies(current_node);
        for(int i = 0; i < ady->size(); i++){
            int adjacent_node = ady->at(i)->getDestination();
            float weight = ady->at(i)->getWeight();
            if (distances[adjacent_node] > distances[current_node] + weight){
                distances[adjacent_node] = distances[current_node] + weight;
                froms[adjacent_node] = current_node;
                pq.emplace(std::pair<int, float>(adjacent_node, distances[adjacent_node]));
            }
        }
    }
    return froms;
}

std::list<int> RNGraph::shortestPath(const int& src, const int& dst) const{
    std::map<int, int> froms = shortestPath(src);
    std::list<int> path;
    path.emplace_front(dst);
    int pred = froms[dst];
    while(pred != RN_NONE){
        path.emplace_front(pred);
        pred = froms[pred];
    }
    return path;
}

void RNGraph::clear(){
    std::map<int, RNAdyacencyList*>::iterator it;
    for(it = graph->begin(); it != graph->end(); it++){
        it->second->clear();
    }
    graph->erase(graph->begin(), graph->end());
}

bool RNGraph::empty() const{
    return (graph->size() == 0);
}