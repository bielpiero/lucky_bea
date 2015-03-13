#include "Matrix.h"

Matrix::Matrix(unsigned int rows, unsigned int cols){
	this->rows = rows;
	this->cols = cols;
	this->data = std::vector<std::vector<float> >(rows, std::vector<float>(cols, 0));
}

Matrix::~Matrix(){

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

Matrix Matrix::operator+(const float& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = rhs + data[i][j];
		}
	}
	return result;
}

Matrix operator+(const float& scalar, Matrix rhs){
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

Matrix Matrix::operator-(const float& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = data[i][j] - rhs;
		}
	}
	return result;
}

Matrix operator-(const float& scalar, Matrix rhs){
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

Matrix Matrix::operator*(const float& rhs){
	Matrix result(this->rows, this->cols);
	for(int i = 0; i < result.rows_size(); i++){
		for (int j = 0; j < result.cols_size(); j++){
			result(i,j) = rhs*data[i][j];
		}
	}
	return result;
}

Matrix operator*(const float& scalar, Matrix rhs){
	return (rhs*scalar);
}

Matrix operator!(Matrix rhs){
	return rhs.inv();
}

Matrix operator~(Matrix rhs){
	return rhs.transpose();
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
	if(determinant == 0){
		throw std::invalid_argument("This Matrix has no inverse");
	}
	Matrix result(this->rows_size(), this->cols_size());
	Matrix cof = cofactor();
	   
	for(int i = 0; i < this->rows_size(); i++){
		for(int j = 0; j < this->cols_size(); j++){
			result(i, j) = cof(i, j) / determinant;
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
						float tempValue = result(i + 1, j);
						result(i + 1, j) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				} else {
					if(result(i, j) < result(i + 1, j)){
						float tempValue = result(i + 1, j);
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
						float tempValue = result(i, j + 1);
						result(i, j + 1) = result(i, j);
						result(i, j) = tempValue;
						change = true;
					}
				} else {
					if(result(i, j) < result(i, j + 1)){
						float tempValue = result(i, j + 1);
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

float Matrix::det(){
	if(this->rows_size() != this->cols_size()){
		throw std::invalid_argument("Invalid matrix dimension. Matrix must be square");
	}
	Matrix self = *this;
	
	return this->det(*this);
}

float Matrix::det(Matrix rhs){

	float determinant = 0;
	if(rhs.rows_size() == 1){
		determinant = rhs(0, 0);
	} else if(rhs.rows_size() == 2){
		determinant = (rhs(0, 0) * rhs(1, 1)) - (rhs(0, 1) * rhs(1, 0));
	} else {

		Matrix minor(rhs.rows_size() - 1, rhs.cols_size() - 1);
		for(int x = 0; x < rhs.rows_size(); x++){
			for(int i = 1; i < rhs.rows_size(); i++){
				int j2 = 0;
				for(int j = 0; j < rhs.rows_size(); j++){
					if(j == x){ continue; }
					minor(i - 1, j2) = rhs(i, j);
					j2++;
				}
			}
			determinant += std::pow(-1.0, x + 2) * rhs(0, x) * det(minor);
		}
	}
	return determinant;
}

Matrix Matrix::cofactor(){
	Matrix self = *this;
	Matrix result = Matrix(this->rows, this->cols);
	
	for(int i = 0; i < this->rows; i++){
		for(int j = 0; j < this->cols; j++){
			result(i, j) = 1;
		}
	}
		
	if(self.rows_size() > 1){
		Matrix bMat(this->rows - 1, this->cols - 1);
		for (int j = 0; j < this->cols; j++){
			for (int i = 0; i < this->rows; i++){
				int i1 = 0;
				for(int ii = 0; ii < this->rows; ii++){
					if(ii != i) {
						int j1 = 0;
						for (int jj = 0; jj < this->cols; jj++){
							if(jj != j){
								bMat(i1, j1) = self(ii, jj);
								j1++;
							}
						}
						i1++;
					}
				}
				float deter = det(bMat);
				result(i, j) = std::pow(-1.0, i + j + 2.0) * deter;
			}
		}
	}
	return result;
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
