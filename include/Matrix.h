#ifndef MATRIX_H
#define MATRIX_H

#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <stdexcept>
#include <iomanip>
#include <stdio.h>

#define MATRIX_ASCENDING 0
#define MATRIX_DESCENDING 1

class Matrix{
	private:
		unsigned int rows;
		unsigned int cols;
		std::vector<std::vector<double> > data;

	public:
		Matrix(unsigned int rows = 1, unsigned int cols = 1);
		virtual ~Matrix();
		void operator=(const Matrix& rhs);
		Matrix operator+(const Matrix& rhs);
		Matrix operator+(const double& rhs);
		friend Matrix operator+(const double& scalar, Matrix rhs);
		
		Matrix operator-(const Matrix& rhs);
		Matrix operator-(const double& rhs);
		friend Matrix operator-(const double& scalar, Matrix rhs);
		
		Matrix operator*(const Matrix& rhs);
		Matrix operator*(const double& rhs);
		friend Matrix operator*(const double& scalar, Matrix rhs);

		friend Matrix  operator !(Matrix m);
		friend Matrix  operator ~(Matrix m);
		
		Matrix operator() (const int row) const;
		
		double& operator() (const int row, const int col);
		const double& operator() (const int row, const int col) const;
		
		Matrix mlDivide(const Matrix& rhs);
		
		Matrix transpose();
		Matrix inv();
		Matrix pInv();
		Matrix eig();
		Matrix roots();
		Matrix abs();
		Matrix chol();
		
		Matrix sort(int mode = MATRIX_ASCENDING);
		Matrix sort_cols(int mode = MATRIX_ASCENDING);
		Matrix sort_rows(int mode = MATRIX_ASCENDING);
		
		static Matrix eye(int dim);
		
		double det();
		void factorizationLU(Matrix& L, Matrix& U);
		void svd(Matrix& u, Matrix& s, Matrix& v);
		
		friend std::ostream& operator<< (std::ostream& osObj, const Matrix& rhs);

		void print();
		
		const size_t rows_size() const;
		const size_t cols_size() const;
};
#endif