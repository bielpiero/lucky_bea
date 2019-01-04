#ifndef RN_GRAPH_H
#define RN_GRAPH_H

#include "RNUtils.h"

class RNGraphEdge{
public:
	RNGraphEdge(){
		this->dst = RN_NONE;
		this->weight = 0.0;
	}
	RNGraphEdge(const int& dst, const float& weight = 1.0){
		this->dst = dst;
		this->weight = weight;
	}
	virtual ~RNGraphEdge(){}

	int getDestination() const { return dst; }
	float getWeight() const { return weight; }

	void setWeight(float weight) { this->weight = weight; }

	bool equals(RNGraphEdge* o){
		return dst == o->dst;
	}

	bool equals(const RNGraphEdge& o){
		return dst == o.dst;
	}
private:
	int dst;
	float weight;
};

class RNAdyacencyList {
public:
	RNAdyacencyList();
	virtual ~RNAdyacencyList();
	void insert(RNGraphEdge* edge);
	void removeEdge(int edgeName);
	bool contains(int edgeName);
	void clear();
	int size() const;
	bool empty() const;
	RNGraphEdge* at(int index);
	
	const std::string toString() const;
private:
	std::list<RNGraphEdge*>* adyacencies;
};

class RNGraph {
public:
	RNGraph();
	virtual ~RNGraph();
	
	void addNode(const int& node);
	int addEdge(const int& src, const int& dst, const float& weight = 1.0);

	void removeNode(int node);
	void removeEdge(int src, int dst);
	void clear();
	bool empty() const;

	std::map<int, float> shortestPath(const int& src) const;
	std::list<int> shortestPath(const int& src, const int& dst) const;
	std::list<int> branchAndBound(const int& src, const int& dst) const;

	RNAdyacencyList* getAdyacencies(int node) const;
	const std::string toString() const;
private:
	void branchAndBound(const int& src, const int& dst, std::list<int>* cerrados, std::list<int>* visitados) const;
private:
	std::map<int, RNAdyacencyList*>* graph;
};
#endif