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

template<class ArgType1>
class RNFunPointer1 : public RNFunPointer{
public:
	virtual ~RNFunPointer1(){}
	virtual void invoke(void) = 0;
	virtual void invoke(ArgType1 p1) = 0;
};

template<class ArgType1, class ArgType2>
class RNFunPointer2 : public RNFunPointer1<ArgType1>{
public:
	virtual ~RNFunPointer2(){}
	virtual void invoke(void) = 0;
	virtual void invoke(ArgType1 p1) = 0;
	virtual void invoke(ArgType1 p1, ArgType2 p2) = 0;
};

template<class ArgType1, class ArgType2, class ArgType3>
class RNFunPointer3 : public RNFunPointer2<ArgType1, ArgType2>{
public:
	virtual ~RNFunPointer3(){}
	virtual void invoke(void) = 0;
	virtual void invoke(ArgType1 p1) = 0;
	virtual void invoke(ArgType1 p1, ArgType2 p2) = 0;
	virtual void invoke(ArgType1 p1, ArgType2 p2, ArgType3 p3) = 0;
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
	virtual void invoke(void) { invokeR(); };

	virtual Ret invokeR(void) = 0;
	virtual Ret invokeR(ArgType1 p1) = 0;
};

template<class Ret, class ArgType1, class ArgType2>
class RNRetFunPointer2 : public RNRetFunPointer1<Ret, ArgType1>{
public:
	~RNRetFunPointer2(){}
	virtual void invoke(void) { invokeR(); };

	virtual Ret invokeR(void) = 0;
	virtual Ret invokeR(ArgType1 p1) = 0;
	virtual Ret invokeR(ArgType1 p1, ArgType2 p2) = 0;
};

template<class Ret, class ArgType1, class ArgType2, class ArgType3>
class RNRetFunPointer3 : public RNRetFunPointer2<Ret, ArgType1, ArgType2>{
public:
	~RNRetFunPointer3(){}
	virtual void invoke(void) { invokeR(); };

	virtual Ret invokeR(void) = 0;
	virtual Ret invokeR(ArgType1 p1) = 0;
	virtual Ret invokeR(ArgType1 p1, ArgType2 p2) = 0;
	virtual Ret invokeR(ArgType1 p1, ArgType2 p2, ArgType2 p3) = 0;
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

template<class T, class ArgType1>
class RNFunPointer1C : public RNFunPointer1<ArgType1>{
protected:
	ArgType1 p1;
	T* obj;
	void (T::*func)(ArgType1);
public:
	RNFunPointer1C() {}
	RNFunPointer1C(T *obj,  void (T::*func)(ArgType1)) {
		this->obj = obj;
		this->func = func;
	}

	RNFunPointer1C(T &obj,  void (T::*func)(ArgType1)) {
		this->obj = &obj;
		this->func = func;
	}

	RNFunPointer1C(T *obj,  void (T::*func)(ArgType1), ArgType1 p1) {
		this->obj = obj;
		this->func = func;
		this->p1 = p1;
	}

	RNFunPointer1C(T &obj,  void (T::*func)(ArgType1), ArgType1 p1) {
		this->obj = &obj;
		this->func = func;
		this->p1 = p1;
	}
	~RNFunPointer1C(){}
	
	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }

	virtual void invoke(void) { (obj->*func)(p1); };
	virtual void invoke(ArgType1 p1) { (obj->*func)(p1); };
	virtual void setArg1(ArgType1 p1) { this->p1 = p1; }
};

template<class T, class ArgType1, class ArgType2>
class RNFunPointer2C : public RNFunPointer2<ArgType1, ArgType2>{
protected:
	ArgType1 p1;
	ArgType2 p2;
	T* obj;
	void (T::*func)(ArgType1, ArgType2);
public:
	RNFunPointer2C() {}
	RNFunPointer2C(T *obj,  void (T::*func)(ArgType1, ArgType2)) {
		this->obj = obj;
		this->func = func;
	}

	RNFunPointer2C(T &obj,  void (T::*func)(ArgType1, ArgType2)) {
		this->obj = &obj;
		this->func = func;
	}

	RNFunPointer2C(T *obj,  void (T::*func)(ArgType1, ArgType2), ArgType1 p1) {
		this->obj = obj;
		this->func = func;
		this->p1 = p1;
	}

	RNFunPointer2C(T &obj,  void (T::*func)(ArgType1, ArgType2), ArgType1 p1) {
		this->obj = &obj;
		this->func = func;
		this->p1 = p1;
	}

	RNFunPointer2C(T *obj,  void (T::*func)(ArgType1, ArgType2), ArgType1 p1, ArgType2 p2) {
		this->obj = obj;
		this->func = func;
		this->p1 = p1;
		this->p2 = p2;
	}

	RNFunPointer2C(T &obj,  void (T::*func)(ArgType1, ArgType2), ArgType1 p1, ArgType2 p2) {
		this->obj = &obj;
		this->func = func;
		this->p1 = p1;
		this->p2 = p2;
	}
	~RNFunPointer2C(){}
	
	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }

	virtual void invoke(void) { (obj->*func)(p1, p2); };
	virtual void invoke(ArgType1 p1) { (obj->*func)(p1, this->p2); };
	virtual void invoke(ArgType1 p1, ArgType2 p2) { (obj->*func)(p1, p2); };
	virtual void setArg1(ArgType1 p1) { this->p1 = p1; }
	virtual void setArg2(ArgType2 p2) { this->p2 = p2; }
};

template<class T, class ArgType1, class ArgType2, class ArgType3>
class RNFunPointer3C : public RNFunPointer3<ArgType1, ArgType2, ArgType3>{
protected:
	ArgType1 p1;
	ArgType2 p2;
	ArgType3 p3;
	T* obj;
	void (T::*func)(ArgType1, ArgType2, ArgType3);
public:
	RNFunPointer3C() {}
	RNFunPointer3C(T *obj,  void (T::*func)(ArgType1, ArgType2, ArgType3)) {
		this->obj = obj;
		this->func = func;
	}

	RNFunPointer3C(T &obj,  void (T::*func)(ArgType1, ArgType2, ArgType3)) {
		this->obj = &obj;
		this->func = func;
	}

	RNFunPointer3C(T *obj,  void (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 p1) {
		this->obj = obj;
		this->func = func;
		this->p1 = p1;
	}

	RNFunPointer3C(T &obj,  void (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 p1) {
		this->obj = &obj;
		this->func = func;
		this->p1 = p1;
	}

	RNFunPointer3C(T *obj,  void (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 p1, ArgType2 p2) {
		this->obj = obj;
		this->func = func;
		this->p1 = p1;
		this->p2 = p2;
	}

	RNFunPointer3C(T &obj,  void (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 p1, ArgType2 p2) {
		this->obj = &obj;
		this->func = func;
		this->p1 = p1;
		this->p2 = p2;
	}

	RNFunPointer3C(T *obj,  void (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 p1, ArgType2 p2, ArgType3 p3) {
		this->obj = obj;
		this->func = func;
		this->p1 = p1;
		this->p2 = p2;
		this->p3 = p3;
	}

	RNFunPointer3C(T &obj,  void (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 p1, ArgType2 p2, ArgType3 p3) {
		this->obj = &obj;
		this->func = func;
		this->p1 = p1;
		this->p2 = p2;
		this->p3 = p3;
	}
	~RNFunPointer3C(){}
	
	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }

	virtual void invoke(void) { (obj->*func)(p1, p2, p3); };
	virtual void invoke(ArgType1 p1) { (obj->*func)(p1, this->p2, this->p3); };
	virtual void invoke(ArgType1 p1, ArgType1 p2) { (obj->*func)(p1, p2, this->p3); };
	virtual void invoke(ArgType1 p1, ArgType2 p2, ArgType3 p3) { (obj->*func)(p1, p2, p3); };
	virtual void setArg1(ArgType1 p1) { this->p1 = p1; }
	virtual void setArg2(ArgType2 p2) { this->p2 = p2; }
	virtual void setArg2(ArgType3 p2) { this->p3 = p3; }
};

template<class Ret, class T>
class RNRetFunPointerC : RNRetFunPointer<Ret>{
protected:
	T* obj;
	Ret (T::*func)(void);
public:
	RNRetFunPointerC() {}

	RNRetFunPointerC(T &obj, Ret (T::*func)(void)){
		this->obj = &obj;
		this->func = func;
	}

	RNRetFunPointerC(T *obj, Ret (T::*func)(void)){
		this->obj = obj;
		this->func = func;
	}

	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }

	virtual Ret invokeR(void) { return (obj->*func)(); }
};

template<class Ret, class T, class ArgType1>
class RNRetFunPointerC1 : public RNRetFunPointer1<Ret, ArgType1>{
protected:
	T* obj;
	Ret (T::*func)(ArgType1);
	ArgType1 arg1;
public:
	RNRetFunPointerC1() {}

	RNRetFunPointerC1(T &obj, Ret (T::*func)(ArgType1)){
		this->obj = &obj;
		this->func = func;
	}

	RNRetFunPointerC1(T &obj, Ret (T::*func)(ArgType1), ArgType1 arg1){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
	}

	RNRetFunPointerC1(T *obj, Ret (T::*func)(ArgType1)){
		this->obj = obj;
		this->func = func;
	}

	RNRetFunPointerC1(T *obj, Ret (T::*func)(ArgType1), ArgType1 arg1){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
	}

	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }
	void setArg1(ArgType1 arg1) { this->arg1 = arg1; }

	virtual Ret invokeR(void) { return (obj->*func)(this->arg1); }
	virtual Ret invokeR(ArgType1 arg1) { return (obj->*func)(arg1); }
};

template<class Ret, class T, class ArgType1, class ArgType2>
class RNRetFunPointerC2 : public RNRetFunPointer2<Ret, ArgType1, ArgType2>{
protected:
	T* obj;
	Ret (T::*func)(ArgType1, ArgType2);
	ArgType1 arg1;
	ArgType2 arg2;
public:
	RNRetFunPointerC2() {}

	RNRetFunPointerC2(T &obj, Ret (T::*func)(ArgType1, ArgType2)){
		this->obj = &obj;
		this->func = func;
	}

	RNRetFunPointerC2(T &obj, Ret (T::*func)(ArgType1, ArgType2), ArgType1 arg1){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
	}

	RNRetFunPointerC2(T &obj, Ret (T::*func)(ArgType1, ArgType2), ArgType1 arg1, ArgType2 arg2){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
		this->arg2 = arg2;
	}

	RNRetFunPointerC2(T *obj, Ret (T::*func)(ArgType1, ArgType2)){
		this->obj = obj;
		this->func = func;
	}

	RNRetFunPointerC2(T *obj, Ret (T::*func)(ArgType1, ArgType2), ArgType1 arg1){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
	}

	RNRetFunPointerC2(T *obj, Ret (T::*func)(ArgType1, ArgType2), ArgType1 arg1, ArgType2 arg2){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
		this->arg2 = arg2;
	}

	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }
	void setArg1(ArgType1 arg1) { this->arg1 = arg1; }
	void setArg2(ArgType2 arg2) { this->arg2 = arg2; }

	virtual Ret invokeR(void) { return (obj->*func)(this->arg1); }
	virtual Ret invokeR(ArgType1 arg1) { return (obj->*func)(arg1, this->arg2); }
	virtual Ret invokeR(ArgType1 arg1, ArgType2 arg2) { return (obj->*func)(arg1, arg2); }
};

template<class Ret, class T, class ArgType1, class ArgType2, class ArgType3>
class RNRetFunPointerC3 : public RNRetFunPointer3<Ret, ArgType1, ArgType2, ArgType3>{
protected:
	T* obj;
	Ret (T::*func)(ArgType1, ArgType2, ArgType3);
	ArgType1 arg1;
	ArgType2 arg2;
	ArgType3 arg3;
public:
	RNRetFunPointerC3() {}

	RNRetFunPointerC3(T &obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3)){
		this->obj = &obj;
		this->func = func;
	}

	RNRetFunPointerC3(T &obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 arg1){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
	}

	RNRetFunPointerC3(T &obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 arg1, ArgType2 arg2){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
		this->arg2 = arg2;
	}

	RNRetFunPointerC3(T &obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 arg1, ArgType2 arg2, ArgType3 arg3){
		this->obj = &obj;
		this->func = func;
		this->arg1 = arg1;
		this->arg2 = arg2;
		this->arg3 = arg3;
	}

	RNRetFunPointerC3(T *obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3)){
		this->obj = obj;
		this->func = func;
	}

	RNRetFunPointerC3(T *obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 arg1){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
	}

	RNRetFunPointerC3(T *obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 arg1, ArgType2 arg2){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
		this->arg2 = arg2;
	}

	RNRetFunPointerC3(T *obj, Ret (T::*func)(ArgType1, ArgType2, ArgType3), ArgType1 arg1, ArgType2 arg2, ArgType3 arg3){
		this->obj = obj;
		this->func = func;
		this->arg1 = arg1;
		this->arg2 = arg2;
		this->arg3 = arg3;
	}

	void setSelf(T *obj){ this->obj = obj; }
	void setSelf(T &obj){ this->obj = &obj; }
	void setArg1(ArgType1 arg1) { this->arg1 = arg1; }
	void setArg2(ArgType2 arg2) { this->arg2 = arg2; }
	void setArg3(ArgType3 arg3) { this->arg3 = arg3; }

	virtual Ret invokeR(void) { return (obj->*func)(this->arg1, this->arg2, this->arg3); }
	virtual Ret invokeR(ArgType1 arg1) { return (obj->*func)(arg1, this->arg2, this->arg3); }
	virtual Ret invokeR(ArgType1 arg1, ArgType2 arg2) { return (obj->*func)(arg1, arg2, this->arg3); }
	virtual Ret invokeR(ArgType1 arg1, ArgType2 arg2, ArgType3 arg3) { return (obj->*func)(arg1, arg2, arg3); }
};

#endif