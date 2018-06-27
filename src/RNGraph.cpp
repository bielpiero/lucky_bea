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

void RNGraph::addEdge(const int& src, const int& dst, const float& weight){
    RNGraphEdge* edge = new RNGraphEdge(dst, weight);
    RNAdyacencyList* adys = NULL;
    std::map<int, RNAdyacencyList*>::iterator it;

    it = graph->find(src);
    if(it != graph->end()){
        adys = it->second;
        adys->insert(edge);
        graph->emplace(src, adys);
    }
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

RNAdyacencyList* RNGraph::getAdyacencies(int node){
    RNAdyacencyList* adys = NULL;
    std::map<int, RNAdyacencyList*>::iterator it;

    it = graph->find(node);
    if(it != graph->end()){
        adys = it->second;
    }
    return adys;
}

void RNGraph::clear(){
    std::map<int, RNAdyacencyList*>::iterator it;
    for(it = graph->begin(); it != graph->end(); it++){
        it->second->clear();
    }
    graph->erase(graph->begin(), graph->end());
}