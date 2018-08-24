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

RNAdyacencyList::iterator RNAdyacencyList::begin(){
    adyacencies->begin();
}

RNAdyacencyList::iterator RNAdyacencyList::end(){
    adyacencies->end();
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
        graph->emplace(it->first, adys);
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
        graph->emplace(src, adys);
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

std::list<int> RNGraph::shortestPath(const int& src, const int& dst) const{
    std::list<int> path;
    std::list<int> selected;
    std::map<int, int> prev;
    std::map<int, float> dist;
    int start = src;
    bool mal = false;
    selected.emplace_back(start);
    dist.emplace(start, 0.0);
    bool finalNodeArrived = std::find(selected.begin(), selected.end(), dst) != selected.end();
    while(not finalNodeArrived and not mal){
        float min = std::numeric_limits<float>::max();
        int m = RN_NONE;
        RNAdyacencyList* ady = this->getAdyacencies(start);
        if(not ady->empty()){
            RNAdyacencyList::iterator it;
            printf("ta aqui llega\n");
            for(it = ady->begin(); it != ady->end(); it++){
                printf("ta aqui llega tambien\n");
                float d = dist.find(start)->second + (*it)->getWeight();
                printf("Node: %d, Dist: %f", start, d);
                float dstN = dist.find((*it)->getDestination()) != dist.end() ? dist.find((*it)->getDestination())->second : std::numeric_limits<float>::max();
                if((d < dstN) and (std::find(selected.begin(), selected.end(), (*it)->getDestination()) != selected.end())){
                    dist.emplace((*it)->getDestination(), d);
                    dstN = dist.find((*it)->getDestination())->second;
                    prev.emplace((*it)->getDestination(), start);
                }
                if((min > dst) and (std::find(selected.begin(), selected.end(), (*it)->getDestination()) != selected.end())){
                    min = dstN;
                    m = (*it)->getDestination();
                }
            }
        } else {
            printf("ady list empty\n");
        }
        if(m != RN_NONE){
            start = m;
            selected.emplace_back(start);
        } else {
            mal = true;
        }
        finalNodeArrived = std::find(selected.begin(), selected.end(), dst) != selected.end();
        printf("finalNodeArrived: %d\n", (int)finalNodeArrived);
    }
    if(not mal){
        int end = dst;
        path.emplace_back(dst);
        while(end != src){
            end = prev[end];
            path.emplace_front(end);
        }
    }
    printf("Exiting shortestPath\n");
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