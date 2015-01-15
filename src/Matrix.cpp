#include "Matrix.h"

Matrix::Matrix(unsigned int rows, unsigned int cols){
	this->rows = rows;
	this->cols = cols;
	this->data = std::vector<std::vector<float> >(rows, std::vector<float>(cols, 0));
}

Matrix::~Matrix(){

}

void Matrix::operator=(const Matrix& rhs){
	data = rhs.data;
}

Matrix Matrix::operator+(const Matrix& rhs){
	if(this->rows != rhs.rows || this->cols != rhs.cols){
		throw std::invalid_argument("Matrices must have same dimension");
	}
	Matrix result(rhs.rows, rhs.cols);
	
	for(int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[i].size(); j++){
			result(i,j) = rhs(i, j) + data[i][j];
		}
	}
	
	return result;
}


Matrix Matrix::operator-(const Matrix& rhs){
	if(this->rows != rhs.rows || this->cols != rhs.cols){
		throw std::invalid_argument("Matrices must have same dimension");
	}
	Matrix result(rhs.rows, rhs.cols);
	
	for(int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[i].size(); j++){
			result(i,j) = rhs(i, j) - data[i][j];
		}
	}
	
	return result;
}


Matrix Matrix::operator*(const Matrix& rhs){
	if(this->cols != rhs.rows){
		throw std::invalid_argument("This matrix A cols must be the same as matrix B rows");
	}
	Matrix result(this->rows, rhs.cols);
	
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			for (int k = 0; k < rhs.rows_size(); k++){
				result(i,j) += data[i][k] * rhs(k, j);
			}
		}
	}
	
	return result;
}

const float& Matrix::operator() (const int row, const int col) const{
	if(row < 0 && row >= data.size() || col < 0 && col >= data[row].size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return data[row][col];
}

float& Matrix::operator() (const int row, const int col){
	float value = NAN;
	if(row < 0 && row >= data.size() || col < 0 && col >= data[row].size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return data[row][col];
}

Matrix Matrix::operator() (const int row) const{
	Matrix rowData(1, data[0].size());
	if(row > -1 && row < data.size()){
		rowData.data[0] = data[row];
	}
	return rowData;
}

Matrix Matrix::operator*(const float& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = rhs*data[i][j];
		}
	}
	return result;
}

Matrix Matrix::mlDivide(const Matrix& rhs){
	Matrix A = *this;
	return ((A.transpose() * A).inv() * A.transpose() * rhs);
}
		
Matrix Matrix::transpose(){
	Matrix result(this->cols, this->rows);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = data[j][i];
		}
	}
	return result;
}

Matrix Matrix::inv(){
	if(this->rows_size() != this->cols_size()){
		throw std::invalid_argument("Invalid matrix dimension. Matrix must be square");
	}
	float determinant = det();
	if(determinant == 0)
		throw std::invalid_argument("This Matrix has no inverse");
	}
}

Matrix Matrix::pInv(){
	Matrix A = *this;
	return ((A.transpose() * A).inv() * A.transpose());
}

Matrix Matrix::eig(){
	
}

Matrix Matrix::roots(){
	return this->eig();
}

Matrix Matrix::svd(){
	
}

float Matrix::det(){
	if(this->rows_size() != this->cols_size()){
		throw std::invalid_argument("Invalid matrix dimension. Matrix must be square");
	}
	
	return this->det(*this);
}

float Matrix::det(Matrix rhs){
	Matrix minor(rhs.rows_size() - 1, rhs.cols_size() - 1);
	float determinant = 0;
	if(rhs.cols_size() == 1){
		return rhs(0, 0);
	} else {
		float s = 1;
		for (int i = 0; i < rhs.cols_size(); i++){
			int mIndex = 0, nIndex = 0;
			for(int j = 0; j < rhs.rows_size(); j++){
				for(int k = 0; k < rhs.cols_size(); k++){
					minor(j, k) = 0;
					if(j != 0 && k != i){
						minor(mIndex, nIndex) = rhs(j, k);
						if(nIndex < (rhs.rows_size() - 2)){
							nIndex++;
						} else {
							nIndex = 0;
							mIndex++;
						}
					}
				}
			}
			determinant += s * (rhs(0, i) * det(minor));
			s = -1 * s;
		}
	}
	return determinant;
}

Matrix Matrix::abs(){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[i].size(); j++){
			result(i,j) = std::abs(data[i][j]);
		}
	}
	return result;
}


const size_t Matrix::rows_size() const{
	return rows;
}

const size_t Matrix::cols_size() const{
	return cols;
}