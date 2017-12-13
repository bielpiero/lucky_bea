#include "Matrix.h"

Matrix::Matrix(unsigned int rows, unsigned int cols){
	this->rows = rows;
	this->cols = cols;
	this->data = std::vector<std::vector<double> >(rows, std::vector<double>(cols, 0));
}

Matrix::~Matrix(){
	for (int i = 0; i < data.size(); i++){
		data.at(i).clear();
	}
	data.clear();
}

void Matrix::operator=(const Matrix& rhs){
	this->rows = rhs.rows;
	this->cols = rhs.cols;
	data = rhs.data;
}

Matrix Matrix::operator+(const Matrix& rhs){
	if(this->rows != rhs.rows || this->cols != rhs.cols){
		throw std::invalid_argument("Matrices must have same dimension");
	}
	Matrix result(rhs.rows, rhs.cols);
	
	for(int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[i].size(); j++){
			result(i,j) = data[i][j] + rhs(i, j);
		}
	}
	
	return result;
}

Matrix Matrix::operator+(const double& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = rhs + data[i][j];
		}
	}
	return result;
}

Matrix operator+(const double& scalar, Matrix rhs){
	return (rhs + scalar);
}


Matrix Matrix::operator-(const Matrix& rhs){
	if(this->rows != rhs.rows || this->cols != rhs.cols){
		throw std::invalid_argument("Matrices must have same dimension");
	}
	Matrix result(rhs.rows, rhs.cols);
	
	for(int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[i].size(); j++){
			result(i,j) = data[i][j] - rhs(i, j);
		}
	}
	
	return result;
}

Matrix Matrix::operator-(const double& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = data[i][j] - rhs;
		}
	}
	return result;
}

Matrix operator-(const double& scalar, Matrix rhs){
	return (scalar + (-1 * rhs));
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

Matrix Matrix::operator*(const double& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = rhs*data[i][j];
		}
	}
	return result;
}

Matrix operator*(const double& scalar, Matrix rhs){
	return (rhs*scalar);
}

Matrix operator!(Matrix rhs){
	return rhs.inv();
}

Matrix operator~(Matrix rhs){
	return rhs.transpose();
}

const double& Matrix::operator() (const int row, const int col) const{
	if(row < 0 && row >= data.size() || col < 0 && col >= data[row].size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return data[row][col];
}

double& Matrix::operator() (const int row, const int col){
	double value = NAN;
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
	Matrix self = *this;
	Matrix I = eye(this->rows_size());
	Matrix result(this->rows_size(), this->cols_size());
	Matrix L;
	Matrix U;
	self.factorizationLU(L, U);

	for (int i = 0; i < this->rows_size(); i++){
		Matrix Z(this->rows_size(), 1);
		for (int j = 0; j < this->rows_size(); j++){
			double sum = 0;
			for (int k = 0; k < j; k++){
				sum += L(j, k) * Z(k, 0);
			}
			Z(j, 0) = (I(i, j) - sum) / L(j, j);
		}
		
		for (int j = this->rows_size() - 1; j >= 0; j--){
			double sum = 0;
			for (int k = j + 1; k < this->rows_size(); k++){
				sum += U(j, k) * result(k, i);
			}
			result(j, i) = (Z(j, 0) - sum) / U(j, j);
		}
	}

	return result;
}

Matrix Matrix::pInv(){
	Matrix A = *this;
	return ((A.transpose() * A).inv() * A.transpose());
}

Matrix Matrix::eig(){
	
}

Matrix Matrix::sort(int mode){
	Matrix self = *this;
	Matrix result = self;
	for(int j = 0; j < this->cols; j++){
		bool change = true;
		while(change){
			change = false;
			for(int i = 0; i < this->rows - 1; i++){
				if(mode == MATRIX_ASCENDING){
					if(result(i, j) > result(i + 1, j)){
						double tempValue = result(i + 1, j);
						result(i + 1, j) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				} else {
					if(result(i, j) < result(i + 1, j)){
						double tempValue = result(i + 1, j);
						result(i + 1, j) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				}
			}
		}
	}
	
	for(int i = 0; i < this->rows; i++){
		bool change = true;
		while(change){
			change = false;
			for(int j = 0; j < this->cols - 1; j++){
				if(mode == MATRIX_ASCENDING){
					if(result(i, j) > result(i, j + 1)){
						double tempValue = result(i, j + 1);
						result(i, j + 1) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				} else {
					if(result(i, j) < result(i, j + 1)){
						double tempValue = result(i, j + 1);
						result(i, j + 1) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				}
			}
		}
	}
	return result;
}

Matrix Matrix::sort_rows(int mode){
	Matrix self = *this;
	Matrix result = self;
	for(int j = 0; j < this->cols; j++){
		bool change = true;
		while(change){
			change = false;
			for(int i = 0; i < this->rows - 1; i++){
				if(mode == MATRIX_ASCENDING){
					if(result(i, j) > result(i + 1, j)){
						double tempValue = result(i + 1, j);
						result(i + 1, j) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				} else {
					if(result(i, j) < result(i + 1, j)){
						double tempValue = result(i + 1, j);
						result(i + 1, j) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				}
			}
		}
	}
	return result;
}

Matrix Matrix::sort_cols(int mode){
	Matrix self = *this;
	Matrix result = self;
	for(int i = 0; i < this->rows; i++){
		bool change = true;
		while(change){
			change = false;
			for(int j = 0; j < this->cols - 1; j++){
				if(mode == MATRIX_ASCENDING){
					if(result(i, j) > result(i, j + 1)){
						double tempValue = result(i, j + 1);
						result(i, j + 1) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				} else {
					if(result(i, j) < result(i, j + 1)){
						double tempValue = result(i, j + 1);
						result(i, j + 1) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				}
			}
		}
	}
	return result;
}

Matrix Matrix::eye(int dim){
	if(dim < 2){
		throw std::invalid_argument("Invalid matrix dimension. It must be higher or equal to two");
	}
	Matrix result(dim, dim);
	for(int i = 0; i < dim; i++){
		for(int j = 0; j < dim; j++){
			if(i == j){
				result(i, j) = 1.0;
			}
		}
	}
	return result;
}

Matrix Matrix::roots(){
	return this->eig();
}

void Matrix::svd(Matrix& u, Matrix& s, Matrix& v){
	u = (*this)*(this->transpose());
}

double Matrix::det(){
	if(this->rows_size() != this->cols_size()){
		throw std::invalid_argument("Invalid matrix dimension. Matrix must be square");
	}
	double determinant = 0;
	Matrix self = *this;
	Matrix L;
	Matrix U;
	self.factorizationLU(L, U);

	double diagL = 1, diagU = 1;
	for(int i = 0; i < self.rows_size(); i++){
		diagL = diagL * L(i, i);
		diagU = diagU * U(i, i);
	}
	determinant = diagL * diagU;
	
	return determinant;
}

void Matrix::factorizationLU(Matrix& L, Matrix& U){
	Matrix rhs = *this;

	L = Matrix(rhs.rows_size(), rhs.cols_size());
	U = eye(rhs.rows_size());	
	double sum = 0;

	for(int j = 0; j < rhs.rows_size(); j++){
		for(int i = j; i < rhs.rows_size(); i++){
			sum = 0;
			for(int k = 0; k < j; k++){
				sum += L(i, k) * U(k, j);
			}
			L(i, j) = rhs(i, j) - sum;
		}

		for(int i = j; i < rhs.rows_size(); i++){
			sum = 0;
			for(int k = 0; k < j; k++){
				sum += L(j, k) * U(k, i);
			}
			U(j, i) = (rhs(j, i) - sum) / L(j, j);
		}
	}
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

std::ostream& operator<<(std::ostream& osObj, const Matrix& rhs){

	for(int i = 0; i < rhs.rows_size(); i++){
		for(int j = 0; j < rhs.cols_size(); j++){
			osObj << "(" << i << ", " << j << "): " << rhs(i, j) << "\t";
		}
		osObj << std::endl;
	}
	return osObj;
}

void Matrix::print(){
	Matrix rhs = *this;
	for(int i = 0; i < rhs.rows_size(); i++){
		for(int j = 0; j < rhs.cols_size(); j++){
			printf("(%d, %d): %0.7f\t", i, j, rhs(i, j));
		}
		printf("\n");
	}
}
