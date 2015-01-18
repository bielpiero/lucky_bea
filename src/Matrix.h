#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <stdexcept>

class Matrix{
	private:
		unsigned int rows;
		unsigned int cols;
		std::vector<std::vector<float> > data;

		float det(Matrix rhs);
	public:
		Matrix(unsigned int rows, unsigned int cols = 1);
		~Matrix();
		void operator=(const Matrix& rhs);
		Matrix operator+(const Matrix& rhs);
		Matrix operator-(const Matrix& rhs);
		Matrix operator*(const Matrix& rhs);
		
		Matrix operator*(const float& rhs);
		
		Matrix operator() (const int row) const;
		
		float& operator() (const int row, const int col);
		const float& operator() (const int row, const int col) const;
		
		Matrix mlDivide(const Matrix& rhs);
		
		Matrix transpose();
		Matrix inv();
		Matrix pInv();
		Matrix eig();
		Matrix roots();
		Matrix abs();
		Matrix cofactor();
		static Matrix eye(int dim);
		
		float det();
		
		void svd(Matrix& u, Matrix& s, Matrix& v);
		
		const size_t rows_size() const;
		const size_t cols_size() const;
};
