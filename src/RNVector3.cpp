#include "RNVector3.h"

RNVector3::RNVector3(){
	data = std::vector<float>(3, 0);
}

RNVector3::~RNVector3(){
	data.clear();
}

RNVector3::RNVector3(const float x, const float y, const float z){
	data = std::vector<float>(3, 0);
	data[0] = x;
	data[1] = y;
	data[2] = z;
}

const float& RNVector3::operator() (const unsigned int pos) const{
	if(pos >= data.size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return data[pos];
}

float& RNVector3::operator() (const unsigned int pos){
	if(pos >= data.size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return data[pos];
}

void RNVector3::operator=(const RNVector3& v){
	data[0] = v(0);
	data[1] = v(1);
	data[2] = v(2);
}

RNVector3 RNVector3::operator+(const RNVector3& v){
	return RNVector3(data[0] + v(0), data[1] + v(1), data[2] + v(2));
}

RNVector3 operator+(const RNVector3& v1, const RNVector3& v2){
	return RNVector3(v1(0) + v2(0), v1(1) + v2(1), v1(2) + v2(2));	
}

RNVector3 operator+(const float& scalar, RNVector3 rhs){
	return (rhs + scalar);
}

RNVector3 RNVector3::operator+(const float& v){
	return RNVector3(data[0] + v, data[1] + v, data[2] + v);
}

RNVector3& RNVector3::operator+=(const RNVector3& v){
	data[0] += v(0);
	data[1] += v(1);
	data[2] += v(2);
	return *this;
}

RNVector3& RNVector3::operator+=(const float& v){
	data[0] += v;
	data[1] += v;
	data[2] += v;
	return *this;
}

RNVector3 RNVector3::operator-() const{
	return RNVector3(-data[0], -data[1], -data[2]);
}

RNVector3 RNVector3::operator-(const RNVector3& v){
	return RNVector3(data[0]-v(0), data[1]-v(1), data[2]-v(2));
}

RNVector3 operator-(const RNVector3& v1, const RNVector3& v2){
	return RNVector3(v1(0) - v2(0), v1(1) - v2(1), v1(2) - v2(2));	
}

RNVector3 operator-(const float& scalar, RNVector3 rhs){
	return (rhs - scalar);
}

RNVector3 RNVector3::operator-(const float& v){
	return RNVector3(data[0] - v, data[1] - v, data[2] - v);
}

RNVector3& RNVector3::operator-=(const RNVector3& v){
	data[0] -= v(0);
	data[1] -= v(1);
	data[2] -= v(2);
	return *this;
}

RNVector3& RNVector3::operator-=(const float& v){
	data[0] -= v;
	data[1] -= v;
	data[2] -= v;
	return *this;
}

RNVector3 RNVector3::operator*(const RNVector3& v){
	return RNVector3(data[0] * v(0), data[1] * v(1), data[2] * v(2));
}

RNVector3 RNVector3::operator*(const float& v){
	return RNVector3(data[0] * v, data[1] * v, data[2] * v);
}

RNVector3 operator*(const float& scalar, RNVector3 rhs){
	return (rhs * scalar);
}

RNVector3 operator*(const RNVector3& v1, const RNVector3& v2){
	return RNVector3(v1(0) * v2(0), v1(1) * v2(1), v1(2) * v2(2));	
}

RNVector3& RNVector3::operator*=(const RNVector3& v){
	data[0] *= v(0);
	data[1] *= v(1);
	data[2] *= v(2);
	return *this;
}

RNVector3& RNVector3::operator*=(const float& v){
	data[0] *= v;
	data[1] *= v;
	data[2] *= v;
	return *this;
}

RNVector3 RNVector3::operator/(const RNVector3& v){
	return RNVector3(data[0] / v(0), data[1] / v(1), data[2] / v(2));
}

RNVector3 operator/(const RNVector3& v1, const RNVector3& v2){
	return RNVector3(v1(0) / v2(0), v1(1) / v2(1), v1(2) / v2(2));	
}

RNVector3 operator/(const float& scalar, RNVector3 rhs){
	return (rhs / scalar);
}

RNVector3 RNVector3::operator/(const float& v){
	return RNVector3(data[0] / v, data[1] / v, data[2] / v);
}

RNVector3& RNVector3::operator/=(const RNVector3& v){
	data[0] /= v(0);
	data[1] /= v(1);
	data[2] /= v(2);
	return *this;
}

RNVector3& RNVector3::operator/=(const float& v){
	data[0] /= v;
	data[1] /= v;
	data[2] /= v;
	return *this;
}

bool RNVector3::operator==(const RNVector3& v) const{
	return ((v(0) == data[0]) and (v(1) == data[1]) and (v(2) == data[2]));
}

bool RNVector3::operator!=(const RNVector3& v) const{
	return !(*this == v);
}

float RNVector3::dot(const RNVector3& v) const{
	return ((data[0] * v(0)) + (data[1] * v(1)) + (data[2] * v(2)));
}

float RNVector3::length2() const{
	return dot(*this);
}

float RNVector3::length() const{
	return std::sqrt(length2());
}

RNVector3 RNVector3::norm(){
	return *this / length();
}

RNVector3 RNVector3::sign() {
	RNVector3 result;
	if(not isZero()){
		result = *this / (length());
	}
	return result;
}

RNVector3 RNVector3::absolute() const{
	return RNVector3(std::abs(data[0]), std::abs(data[1]), std::abs(data[2]));
}

float RNVector3::angle(const RNVector3& v) const{
	float s = std::sqrt(length2() * v.length2());
	return std::acos(dot(v) / s);
}

float RNVector3::distance(const RNVector3& v) const{
	return (v - *this).length();
}

float RNVector3::distance2(const RNVector3& v) const{
	return (v - *this).length2();
}

RNVector3 RNVector3::cross(const RNVector3& v) const{
	return RNVector3(data[1] * v(2) - data[2] * v(1),
					data[2] * v(0) - data[0] * v(2),
					data[0] * v(1) - data[1] * v(0));
}

float RNVector3::getX() const{
	return data[0];
}

float RNVector3::getY() const{
	return data[1];
}

float RNVector3::getZ() const{
	return data[2];
}

void RNVector3::setX(float x){
	data[0] = x;
}

void RNVector3::setY(float y){
	data[1] = y;
}

void RNVector3::setZ(float z){
	data[2] = z;
}

void RNVector3::setZero(){
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
}

bool RNVector3::isZero(){
	return ((0.0 == data[0]) and (0.0 == data[1]) and (0.0 == data[2]));
}

const char* RNVector3::toString() const{
	char* buffer = new char[255];
	std::sprintf(buffer, "{x: %0.6f, y: %0.6f, z: %0.6f}", data[0], data[1], data[2]);
	return buffer;
}