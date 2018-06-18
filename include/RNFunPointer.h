#ifndef RN_FUN_POINTER_H
#define RN_FUN_POINTER_H

#include <iostream>
#include <string>
#include <cstring>

class RNFunPointer{
public:
	virtual ~RNFunPointer(){}
	const char* getName() { return name.c_str(); }
	void setName(const char* name) { this->name = std::string(name); }
	virtual void invoke(void) = 0;
protected:
	std::string name;
};

template<class T>
class RNFunPointerC : public RNFunPointer{
protected:
	T* obj;
	void (T::*func)(void);
public:
	RNFunPointerC() {}
	RNFunPointerC(T &obj,  void (T::*func)(void)) {
		this->obj = &obj;
		this->func = func;
	}
	RNFunPointerC(T *obj,  void (T::*func)(void)) {
		this->obj = obj;
		this->func = func;
	}
	~RNFunPointerC(){}
	
	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }

	virtual void invoke(void) { (obj->*func)(); };
};

template<class Ret>
class RNRetFunPointer : public RNFunPointer{
public:
	~RNRetFunPointer(){}
	virtual void invoke(void) { invokeR(); };

	virtual Ret invokeR(void) = 0;
};

template<class Ret, class ArgType1>
class RNRetFunPointer1 : public RNRetFunPointer<Ret>{
public:
	~RNRetFunPointer1(){}
	virtual Ret invokeR(ArgType1 arg1) = 0;

	virtual Ret invokeR(void) = 0;
};

template<class Ret, class T>
class RNRetFunPointerClass : RNRetFunPointer<Ret>{
protected:
	T* obj;
	Ret (T::*func)(void);
public:
	RNRetFunPointerClass() {}

	RNRetFunPointerClass(T &obj, Ret (T::*func)(void)){
		this->obj = &obj;
		this->func = func;
	}

	RNRetFunPointerClass(T *obj, Ret (T::*func)(void)){
		this->obj = obj;
		this->func = func;
	}

	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }

	virtual Ret invokeR(void) { return (obj->*func)(); }
};

template<class Ret, class T, class ArgType1>
class RNRetFunPointerClass1 : public RNRetFunPointer1<Ret, ArgType1>{
protected:
	T* obj;
	Ret (T::*func)(ArgType1);
	ArgType1 arg1;
public:
	RNRetFunPointerClass1() {}

	RNRetFunPointerClass1(T &obj, Ret (T::*func)(ArgType1)){
		this->obj = &obj;
		this->func = func;
	}

	RNRetFunPointerClass1(T &obj, Ret (T::*func)(ArgType1), ArgType1 arg1){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
	}

	RNRetFunPointerClass1(T *obj, Ret (T::*func)(ArgType1)){
		this->obj = obj;
		this->func = func;
	}

	RNRetFunPointerClass1(T *obj, Ret (T::*func)(ArgType1), ArgType1 arg1){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
	}

	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }
	void setParameter1(ArgType1 arg1) { this->arg1 = arg1; }

	virtual Ret invokeR(void) { return (obj->*func)(this->arg1); }
	virtual Ret invokeR(ArgType1 arg1) { return (obj->*func)(arg1); }
};

#endif