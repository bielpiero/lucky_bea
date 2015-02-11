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

std::vector<float>& operator() (const int row){
	if(row < 0 && row >= data.size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return (data[row]);
}

const std::vector<float>& operator() (const int row) const{
	if(row < 0 && row >= data.size()){
		throw std::invalid_argument("Invalid subscripting dimension");
	}
	return (data[row]);
}

//Matrix Matrix::operator() (const int row) const{
//	Matrix rowData(1, data[0].size());
//	if(row > -1 && row < data.size()){
//		rowData.data[0] = data[row];
//	}
//	return rowData;
//}

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
	if(determinant == 0){
		throw std::invalid_argument("This Matrix has no inverse");
	}
	Matrix result(this->rows_size(), this->cols_size());
	Matrix cof = cofactor();
	   
	for(int i = 0; i < this->rows_size(); i++){
		for(int j = 0; j < this->cols_size(); j++){
			result(j, i) = cof(i, j) / determinant;
		}
	}
}

Matrix Matrix::pInv(){
	Matrix A = *this;
	return ((A.transpose() * A).inv() * A.transpose());
}

Matrix Matrix::eig(){
	
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

Matrix Matrix::cofactor(){
	Matrix result(this->rows, this->cols);
	
	if(result.rows_size() != result.cols_size()){
		return result;
	} else if(result.rows_size() < 2) {
		return result;
	} else if(result.rows_size() == 2) {
		result(0, 0) = data[1][1];
		result(0, 1) = -data[1][0];
		result(1, 0) = -data[0][1];
		result(1, 1) = data[0][0];
		
	} else {
		
		int DIM = result.rows_size();
		Matrix ***temp = new Matrix**[DIM];
		for(int i = 0; i < DIM; i++){
			temp[i] = new Matrix*[DIM];
		}
		for(int i = 0; i < DIM; i++){
			for(int j = 0; j < DIM; j++){
				temp[i][j] = new Matrix(DIM - 1,DIM - 1);
			}
		}
		 

		
		
		for(int k1 = 0; k1 < DIM; k1++){  
			for(int k2 = 0; k2 < DIM; k2++){
				int i1 = 0;
				for(int i = 0; i < DIM; i++){
					int j1 = 0;
					for(int j = 0; j < DIM; j++){
						if(k1 == i || k2 == j){
							continue;
						}
						temp[k1][k2]->data[i1][j1++] = data[i][j];
					}
					if(k1 != i){
						i1++;
					}
				}
			}
		}

		bool flagPositive = true;
 		for(int k1 = 0; k1 < DIM; k1++){  
			flagPositive = ( (k1 % 2) == 0);

			for(int k2 = 0; k2 < DIM; k2++){
				if(flagPositive){
					result(k1, k2) = temp[k1][k2]->det();
					flagPositive = false;
				} else {
					result(k1, k2) = -temp[k1][k2]->det();
					flagPositive = true;
				}
			}
		   
		}

		for(int i = 0; i < DIM; i++)
			for(int j = 0; j < DIM; j++)
				delete temp[i][j];

		for(int i = 0; i < DIM; i++)
			delete [] temp[i];

		delete [] temp;
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
