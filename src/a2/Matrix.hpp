#ifndef _MATRIX_HPP_
#define _MATRIX_HPP_

/**
 * Lightweight Matrix Lirary
 * Clark Zhang
 * czsoup@umich.edu
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <iostream>
#include <cmath>

 // TODO: check if matrix is singular in inverse

template <class T>
class Matrix {
private:
	int32_t _rows;
	int32_t _cols;
	T* _data;
	T _threshold;

public:
	///////////////////////////////////
	// CONSTRUCTORS AND DESTRUCTORS
	///////////////////////////////////

	/**
	 * @brief Basic constructor
	 * 
	 * @param rows number of rows, must be > 0
	 * @param cols number of columns, must be > 0
	 * @param threshold for floating point operations, 
	 * if value < threshold, then we count it as 0
	 */
	Matrix(int32_t rows, int32_t cols, T threshold = 1.0e-6);

	/**
	 * @brief Constructor that takes in data
	 * @details data must be in row first format
	 * Ex: (numbers are the indices of the array)
	 * [0 1 2 3]
	 * [4 5 6 7]
	 * data will be copied into matrix and must be of length rows * cols
	 * 
	 * @param rows number of rows, must be > 0
	 * @param cols number of cols, must be > 0
	 * @param data data to fill the matrix with
	 * @param threshold threshold for floating point operations
	 */
	Matrix(int32_t rows, int32_t cols, T* data, T threshold = 1.0e-6);

	/**
	 * @brief Copy constructor
	 */
	Matrix(const Matrix<T>& other);

	/**
	 * @brief Move constructor
	 */
	Matrix(Matrix<T>&& other);

	/**
	 * @brief Destructor
	 */
	~Matrix();

	///////////////////////////////////
	// BASIC AND ARITHMATIC OPERATIONS
	///////////////////////////////////

	/**
	 * @brief Assignment operator
	 * @details makes a deep copy
	 */
	Matrix<T>& operator=(const Matrix<T>& other);

	/**
	 * @brief Move assignment operator
	 */
	Matrix<T>& operator=(Matrix<T>&& other);

	/**
	 * @brief checks equality. elements and size must be the same
	 */
	bool operator==(const Matrix<T>& other);

	/**
	 * @brief gets reference to an element
	 * 
	 * @param row the row number (zero indexed)
	 * @param col the col number (zero indexed)
	 */
	T& operator()(int32_t row, int32_t col);

	/**
	 * @brief gets copy of an element
	 * 
	 * @param row the row number (zero indexed)
	 * @param col the col number (zero indexed)
	 */
	T operator()(int32_t row, int32_t col) const;

	/**
	 * @brief returns reference to an element
	 * @details you can give the specific index (row oriented)
	 * useful for vectors or for iteration accross the matrix through each row
	 */
	T& operator()(int32_t index);

	T operator()(int32_t index) const;

	/**
	 * @brief checks if an index is in bounds
	 */
	bool validIndex(int32_t row, int32_t col) const;

	/**
	 * @brief computes dest = A + B
	 * @details dest can be the same as A or B
	 */
	static void add(Matrix<T>& dest, const Matrix<T>& A, const Matrix<T>& B);

	/**
	 * @brief computes dest = src + scalar
	 * @details dest can be the same as src
	 */
	static void add(Matrix<T>& dest, const Matrix<T>& src, T scalar);

	/**
	 * @brief computes dest = A - B
	 * @details dest can be the same as A or B
	 */
	static void subtract(Matrix<T>& dest, const Matrix& A, const Matrix& B);

	/**
	 * @brief computes dest = A * B
	 * @details dest can NOT be the same as A or B
	 */
	static void multiply(Matrix<T>& dest, const Matrix<T>& A, const Matrix<T>& B);

	/**
	 * @brief computes dest = src * scalar
	 * @details dest can be the same as src
	 */
	static void multiply(Matrix<T>& dest, const Matrix<T>& src, T scalar);

	/**
	 * @brief overloaded addition operator
	 */
	Matrix<T> operator+(const Matrix<T>& other) const;

	/**
	 * @brief addition operator
	 */
	friend Matrix<T> operator+(const Matrix<T>& A, T scalar) {
		Matrix<T> ret(A._rows, A._cols);
		Matrix<T>::add(ret, A, scalar);
		return ret;
	}

	/**
	 * @brief addition operator
	 */
	friend Matrix<T> operator+(T scalar, const Matrix<T>& A) {
		return A + scalar;
	}

	/**
	 * @brief subtraction operator
	 */
	Matrix<T> operator-(const Matrix<T>& A) const;

	/**
	 * @brief subtraction operator
	 */
	friend Matrix<T>operator-(const Matrix<T>& A, T scalar) {
		return A + (-scalar);
	}

	/**
	 * @brief multiplication operator
	 * does *this * A
	 */
	Matrix<T> operator*(const Matrix<T>& A) const;

	friend Matrix<T> operator*(const Matrix<T>& A, T scalar) {
		return Matrix<T>(A) *= scalar;
	}

	friend Matrix<T> operator*(T scalar, const Matrix<T>& A) {
		return A * scalar;
	}

	/**
	Addition assignment operator
	*/
	Matrix<T>& operator+=(const Matrix<T>& A);

	/**
	 * @brief addition assignment
	 * adds a scalar to each element
	 */
	Matrix<T>& operator+=(T scalar);

	/**
	 * @brief subtraction assignment operator
	 */
	Matrix<T>& operator-=(const Matrix<T>& A);

	/**
	 * @brief subtraction assignment operator
	 * subtracts a scalar from each element
	 */
	Matrix<T>& operator-=(T scalar);

	/**
	 * @brief multiplication assignment operator
	 * right multiplies by A: *this = *this * A;
	 */
	Matrix<T>& operator*=(const Matrix<T>& A);

	/**
	 * @brief multiplication assignment operator
	 * multiplies a scalar to each element
	 */
	Matrix<T>& operator*=(T scalar);

	/**
	 * @brief returns number of rows in matrix
	 */
	int32_t rows() const;

	/**
	 * @brief returns number of columns in matrix
	 */
	int32_t cols() const;

	///////////////////////////////////
	// ELEMENTARY ROW OPERATIONS
	///////////////////////////////////

	/**
	 * @brief multiplies a row by a scalar
	 * 
	 * @param row index of row to multiply
	 * @param scalar scalar to multiply by
	 */
	void rowMultiply(int32_t row, T scalar);

	/**
	 * @brief multiplies a column by a scalar
	 * 
	 * @param col index of column to multiply
	 * @param scalar scalar to multiply by
	 */
	void colMultiply(int32_t col, T scalar);

	/**
	 * @brief swaps two rows
	 * 
	 * @param row1 index of first row
	 * @param row2 index of second row
	 */
	void rowSwap(int32_t row1, T row2);

	/**
	 * @brief swaps two columns
	 * 
	 * @param col1 index of first column
	 * @param col2 index of second column
	 */
	void colSwap(int32_t col1, int32_t col2);

	/**
	 * @brief adds row2 * scalar to row1
	 * 
	 * @param row1 index of row to be added to
	 * @param row2 index of row to add
	 * @param scalar scalar to multiply row2 by
	 */
	void rowAdd(int32_t row1, int32_t row2, T scalar);

	///////////////////////////////////
	// CONVINIENCE FUNCTIONS
	///////////////////////////////////

	/**
	 * @brief makes matrix into identity
	 * @details matrix must be a square matrix
	 */
	void identity();

	/**
	 * @brief fills the matrix with a constant value
	 */
	void fill(T value);

	/**
	 * @brief swaps the data of two matricies of the same size
	 * @details much faster than temp = A; A = B; B = temp;
	 */
	static void swap(Matrix<T>& A, Matrix<T>& B);

	///////////////////////////////////
	// MATRIX OPERATIONS
	///////////////////////////////////

	/**
	 * @brief takes the transpose of src and puts it into dest
	 * @details src can not be the same matrix as dest
	 * 
	 * @param dest matrix to store transpose
	 * @param src matrix to take transpose of
	 */
	static void transpose(Matrix<T>& dest, const Matrix<T>& src);

	/**
	 * @brief reduces current matrix to reduced row echelon form
	 */
	void rowReduce();

	/**
	 * @brief takes inverse of src and stores into dest
	 * @details src and dest can not be same matrices
	 * NOTE: undefined behavior if matrix is singular
	 */
	static void inverse(Matrix<T>& dest, const Matrix<T>& src);

	/**
	 * @brief takes the inverse of src and stores into dest
	 * @details src and dest can not be same matrices
	 * temp is used in calculation of inverse
	 * use this over inverse(dest, src) when it is being called
	 * a lot of times on same sized matrices, so the function
	 * doesn't have to keep allocating and deallocating the same matrix
	 * NOTE: undefined behavior if matrix is singular
	 */
	static void inverse(Matrix<T>& dest, const Matrix<T>& src, Matrix<T>& temp);

	/**
	 * @brief Solves A * x = b
	 * @details matrices must be appropriately sized
	 */
	static void solve(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x);

	/**
	 * @brief Solves A * x = b
	 * @details uses temp matrix for storage during calculation 
	 * if called many times, is faster than solve(A, b, x)
	 * temp must be of size (A.rows(), A.cols())
	 */
	static void solve(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x, Matrix<T>& temp);

	/**
	 * @brief solves A * x = b using LU decomposition
	 */
	static void solveLU(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x);

	/**
	 * @brief solves least squares Ax = b
	 */
	static void leastSquares(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x);
	
	/**
	 * @brief takes LU decomposition of current matrix
	 * 
	 * @param lower matrix to store lower matrix
	 * @param upper matrix to store upper matrix
	 */
	void decompLU(Matrix& lower, Matrix& upper) const;

	/**
	 * @brief prints matrix in pretty format
	 * @details the stream operator '<<' must be defined for
	 * type T for this function to work
	 */
	void print() const;

private:
	/**
	 * @brief row reduces the matrix while
	 * simultaneously doing the same operations on "other"
	 * @param other other matrix to do operations on
	 */
	void mirrorRowReduce(Matrix<T>& other);

	// we can make this more stable by not making the pivot one (I think?)
	/**
	 * @brief row reduces to make lower part of matrix zero while
	 * simultaneously doing the same operations on "other"
	 * @param other other other matrix to do operations on
	 * can be nullptr
	 */
	void rowReduceLower(Matrix<T>* other);

	/**
	 * @brief row reduces to make higher part of matrix zero while
	 * simultaneously doing the same operations on "other"
	 * @param other other other matrix to do operations on
	 * can be nullptr
	 */
	void rowReduceHigher(Matrix<T>* other);
};

template <class T>
Matrix<T>::Matrix(int32_t rows, int32_t cols, T threshold) :
	_rows(rows), _cols(cols), _threshold(threshold) {
	assert(_rows > 0);
	assert(_cols > 0);

	_data = new T[rows * cols];
}

template <class T>
Matrix<T>::Matrix(int32_t rows, int32_t cols, T* data, T threshold) :
	_rows(rows), _cols(cols), _threshold(threshold) {
	assert(_rows > 0);
	assert(_cols > 0);

	_data = new T[rows * cols];
	memcpy(_data, data, _rows * _cols * sizeof(T));
}

template <class T>
Matrix<T>::Matrix(const Matrix<T>& other) :
	_rows(other._rows), _cols(other._cols) {
	assert(_rows > 0);
	assert(_cols > 0);

	int32_t size = _rows * _cols;
	_data = new T[size];
	for (int32_t i = 0; i < size; ++i) {
		_data[i] = (other._data)[i];
	}
}

template <class T>
Matrix<T>::Matrix(Matrix<T>&& other) {
	_rows = other._rows;
	_cols = other._cols;
	_data = other._data;

	other._data = nullptr;
}

template <class T>
Matrix<T>::~Matrix() {
	if (_data != nullptr) {
		delete[] _data;
	}
}

template <class T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& other) {
	// check for self assignment
	if (&other == this) {
		return *this;
	}

	int32_t size = other._rows * other._cols;

	// check if we need to allocate a new array
	if ((int32_t)_rows * (int32_t)_cols != size) {
		delete[] _data;
		_data = new T[size];
	}
	_rows = other._rows;
	_cols = other._cols;
	for (int32_t i = 0; i < size; ++i) {
		_data[i] = (other._data)[i];
	}
	return *this;
}

template <class T>
Matrix<T>& Matrix<T>::operator=(Matrix<T>&& other) {
	// free old data
	delete[] _data;

	// pillage the village!
	_rows = other._rows;
	_cols = other._cols;
	_data = other._data;

	// and raze it to the ground!
	other._data = nullptr;

	return *this;
}

template <class T>
bool Matrix<T>::operator==(const Matrix<T>& other) {
	if (other._rows != _rows || other._cols != _cols) {
		return false;
	}

	int32_t size = _rows * _cols;
	for (int32_t i = 0; i < size; ++i) {
		if (_data[i] != other._data[i]) {
			return false;
		}
	}
	return true;
}

template <class T>
T& Matrix<T>::operator()(int32_t row, int32_t col) {
	return _data[row * _cols + col];
}

template <class T>
T Matrix<T>::operator()(int32_t row, int32_t col) const {
	return _data[row * _cols + col];
}

template <class T>
T& Matrix<T>::operator()(int32_t index) {
	return _data[index];
}

template <class T>
T Matrix<T>::operator()(int32_t index) const {
	return _data[index];
}

template <class T>
bool Matrix<T>::validIndex(int32_t row, int32_t col) const {
	return row < _rows && row >= 0 && col < _cols && col >= 0;
}

template <class T>
void Matrix<T>::add(Matrix<T>& dest, const Matrix<T>& A, const Matrix<T>& B) {
	assert(A._rows == B._rows);
	assert(A._rows == dest._rows);
	assert(A._cols == B._cols);
	assert(A._cols == dest._cols);

	int32_t size = A._rows * A._cols;
	for (int32_t i = 0; i < size; ++i) {
		dest._data[i] = A._data[i] + B._data[i];
	}
}

template <class T>
void Matrix<T>::add(Matrix<T>& dest, const Matrix<T>& src, T scalar) {
	assert(dest._rows = src._rows);
	assert(dest._cols = src._cols);

	int32_t size = src._rows * src._cols;
	for (int32_t i = 0; i < size; ++i) {
		dest._data[i] = src._data[i] + scalar;
	}		
}

template <class T>
void Matrix<T>::subtract(Matrix<T>& dest, const Matrix<T>& A, const Matrix<T>& B) {
	assert(A._rows == B._rows);
	assert(A._rows == dest._rows);
	assert(A._cols == B._cols);
	assert(A._cols == dest._cols);

	int32_t size = A._rows * A._cols;
	for (int32_t i = 0; i < size; ++i) {
		dest._data[i] = A._data[i] - B._data[i];
	}
}

template <class T>
void Matrix<T>::multiply(Matrix<T>& dest, const Matrix<T>& A, const Matrix<T>& B) {
	assert(A._cols == B._rows);
	assert(dest._rows == A._rows);
	assert(dest._cols == B._cols);
	assert(dest._data != nullptr);
	assert(A._data != nullptr);
	assert(B._data != nullptr);

	float sum;
	int32_t sum_index = 0;
	int32_t row_index = 0;
	int32_t col_index;
	for (int32_t i = 0; i < dest._rows; ++i) {
		for (int32_t j = 0; j < dest._cols; ++j) {
			sum = 0;
			col_index = j;
			for (int32_t k = 0; k < A._cols; ++k) {
				sum += A._data[row_index + k] * B._data[col_index];
				col_index += dest._cols;
			}
			dest._data[sum_index++] = sum;
		}
		row_index += A._cols;
	}
}

template <class T>
void Matrix<T>::multiply(Matrix<T>& dest, const Matrix<T>& src, T scalar) {
	assert(dest._rows = src._rows);
	assert(dest._cols = src._cols);

	int32_t size = src._rows * src._cols;
	for (int32_t i = 0; i < size; ++i) {
		dest._data[i] = src._data[i] * scalar;
	}
}

template <class T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& other) const {
	return Matrix<T>(*this) += other;
}

template <class T>
Matrix<T>& Matrix<T>::operator+=(const Matrix<T>& A) {
	add(*this, *this, A);
	return *this;
}

template <class T>
Matrix<T>& Matrix<T>::operator+=(T scalar) {
	add(*this, *this, scalar);
	return *this;
}

template <class T>
Matrix<T>& Matrix<T>::operator-=(const Matrix<T>& A) {
	subtract(*this, *this, A);
	return *this;
}

template <class T>
Matrix<T>& Matrix<T>::operator-=(T scalar) {
	add(*this, *this, -scalar);
	return *this;
}

template <class T>
Matrix<T>& Matrix<T>::operator*=(const Matrix<T>& A) {
	Matrix<T> temp(*this);
	multiply(*this, temp, A);
	return this;
}

template <class T>
Matrix<T>& Matrix<T>::operator*=(T scalar) {
	multiply(*this, *this, scalar);
	return *this;
}

template <class T>
int32_t Matrix<T>::rows() const {
	return _rows;
}

template <class T>
int32_t Matrix<T>::cols() const {
	return _cols;
}

template <class T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& A) const {
	Matrix ret(A._rows, A._cols);
	subtract(ret, *this, A);
	return ret;
}

template <class T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& A) const {
	Matrix ret(this->_rows, A._cols);
	multiply(ret, *this, A);
	return ret;
}

template <class T>
void Matrix<T>::rowMultiply(int32_t row, T scalar) {
	assert(row < _rows);
	assert(_data != nullptr);

	int32_t start = row * _cols;
	int32_t end = start + _cols;
	for (int32_t i = start; i < end; ++i) {
		_data[i] *= scalar;
	}
}

template <class T>
void Matrix<T>::colMultiply(int32_t col, T scalar) {
	assert(col < _cols);
	assert(_data != nullptr);

	int32_t start = col;
	int32_t end = col + (_rows * _cols);
	for (int32_t i = start; i < end; i += _cols) {
		_data[i] *= scalar;
	}
}

template <class T>
void Matrix<T>::rowSwap(int32_t row1, T row2) {
	assert(row1 < _rows);
	assert(row2 < _rows);
	assert(_data != nullptr);

	int32_t row1_end = row1 * _cols +_cols;
	for (int32_t row1_index = row1 * _cols, row2_index = row2 * _cols; 
		row1_index < row1_end; ++row1_index) {
		T temp = _data[row1_index];
		_data[row1_index] = _data[row2_index];
		_data[row2_index] = temp;
		++row2_index;
	}	
}

template <class T>
void Matrix<T>::colSwap(int32_t col1, int32_t col2) {
	assert(col1 < _cols);
	assert(col2 < _cols);
	assert(_data != nullptr);

	for (int32_t rowOffset = 0; rowOffset <= _cols * (_rows - 1); rowOffset += _cols) {
		T temp = _data[rowOffset + col1];
		_data[rowOffset + col1] = _data[rowOffset + col2];
		_data[rowOffset + col2] = temp;
	}
}

template <class T>
void Matrix<T>::rowAdd(int32_t row1, int32_t row2, T scalar) {
	assert(row1 < _rows);
	assert(row2 < _rows);
	assert(_data != nullptr);

	int32_t row1_end = row1 * _cols +_cols;
	for (int32_t row1_index = row1 * _cols, row2_index = row2 * _cols; 
		row1_index < row1_end; ++row1_index) {
		_data[row1_index] += scalar * _data[row2_index];
		++row2_index;
	}	
}

template <class T>
void Matrix<T>::identity() {
	assert(_rows == _cols);
	assert(_data != nullptr);

	int32_t size = _rows * _cols;
	int32_t one_index = 0;
	for (int32_t i = 0; i < size; ++i) {
		if (i == one_index) {
			_data[i] = 1;
			one_index += _cols + 1;
		} else {
			_data[i] = 0;
		}
	}
}

template <class T>
void Matrix<T>::fill(T value) {
	assert(_data != nullptr);

	int32_t size = _rows * _cols;
	for (int32_t i = 0; i < size; ++i) {
		_data[i] = value;
	}
}

template <class T>
void Matrix<T>::swap(Matrix<T>& A, Matrix<T>& B) {
	assert(A._rows == B._rows);
	assert(A._cols == B._cols);

	T* temp = A._data;
	A._data = B._data;
	B._data = temp;
}

template <class T>
void Matrix<T>::transpose(Matrix<T>& dest, const Matrix<T>& src) {
	assert(src._rows == dest._cols);
	assert(src._cols == dest._rows);

	for (int32_t i = 0; i < src._rows; ++i) {
		for (int32_t j = 0; j < src._cols; ++j) {
			dest(j, i) = src(i, j);
		}
	}
}

template <class T>
void Matrix<T>::rowReduce() {
	// row reduces into a (non-reduced) row echelon form
	rowReduceLower(nullptr);

	// row reduces to reduced row echelon form
	rowReduceHigher(nullptr);
}

template <class T>
void Matrix<T>::inverse( Matrix<T>& dest, const Matrix<T>& src) {
	Matrix<T> temp(src._rows, src._rows);
	inverse(dest, src, temp);
}

template <class T>
void Matrix<T>::inverse(Matrix<T>& dest, const Matrix<T>& src, Matrix<T>& temp) {
	assert(src._rows == src._cols);
	assert(dest._rows == dest._cols);
	assert(src._rows == dest._rows);
	assert(src._rows == temp._rows);
	assert(temp._rows == temp._cols);
	assert(src._data != nullptr);
	assert(dest._data != nullptr);
	assert(temp._data != nullptr);

	temp = src;
	dest.identity();
	temp.mirrorRowReduce(dest);
}

template <class T>
void Matrix<T>::solve(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x) {
	Matrix temp(A._rows, A._cols);
	solve(A, b, x, temp);
}

template <class T>
void Matrix<T>::solve(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x, Matrix<T>& temp) {
	assert(A._rows == A._cols);
	assert(A._rows == b._rows);
	assert(A._rows == x._rows);
	assert(b._cols == 1);
	assert(x._cols == 1);
	assert(temp._rows == temp._cols);
	assert(temp._rows == A._rows);
	assert(b._data != nullptr);
	assert(A._data != nullptr);
	assert(x._data != nullptr);
	assert(temp._data != nullptr);

	temp = A;
	x = b;
	temp.mirrorRowReduce(x);
}

template <class T>
void Matrix<T>::solveLU(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x) {
	assert(A._rows == A._cols);
	assert(A._rows == b._rows);
	assert(A._rows == x._rows);
	assert(b._cols == 1);
	assert(x._cols == 1);
	assert(b._data != nullptr);
	assert(A._data != nullptr);
	assert(x._data != nullptr);

	Matrix<T> lower(A._rows, A._rows);
	Matrix<T> upper(A._rows, A._rows);

	// gets LU decomposition of A
	A.decompLU(lower, upper);

	// solves L * y = b
	Matrix y(b);
	lower.rowReduceLower(&y);

	// solves Ux = y
	x = y;
	upper.rowReduceHigher(&x);
}

template <class T>
void Matrix<T>::leastSquares(const Matrix<T>& A, const Matrix<T>& b, Matrix<T>& x) {
	Matrix<T> AT(A._cols, A._rows);
	Matrix<T>::transpose(AT, A);
	Matrix<T> newA = AT * A;
	Matrix<T> newB = AT * b;
	solve(newA, newB, x);
}

template <class T>
void Matrix<T>::decompLU(Matrix& lower, Matrix& upper) const {
	assert(lower._rows == _rows);
	assert(lower._cols == _rows);
	assert(upper._rows == _rows);
	assert(upper._cols == _cols);
	assert(_data != nullptr);
	assert(lower._data != nullptr);
	assert(upper._data != nullptr);

	memcpy(upper._data, _data, _rows * _cols * sizeof(T));

	// following code is basically copied from rowReduceLower()
	// with tiny modifications to save data to form the lower triangular matrix

	// column the pivot is in 
	uint8_t pivot = 0;
	for (uint8_t i = 0; i < _cols; ++i) {
		bool zero_col = true;
		for (uint8_t j = pivot; j < _rows; ++j) {
			// searches down the column for first non-zero entry
			// where non-zero means greater than threshold values
			float element = std::fabs(upper(j, i));
			if (element > _threshold) {
				if (pivot != j) {
					// swap to make the pivot non-zero
					upper.rowSwap(j, pivot);
				}
				zero_col = false;
				break;
			} else if (element != 0) {
				// set it to zero if its not
				upper(j, i) = 0;
			}
		}

		// if column is all zeroes, skip to next loop
		if (zero_col) {
			continue;
		}

		// sets entries before pivot to 0
		for (uint8_t k = 0; k < pivot; ++k) {
			lower(k, pivot) = 0;
		}

		lower(pivot, pivot) = upper(pivot, i);

		// make the pivot = 1
		upper.rowMultiply(pivot, 1.0 / upper(pivot, i));

		// make all entries under the pivot = 0
		for (uint8_t k = pivot + 1; k < _rows; ++k) {
			lower(k, pivot) = upper(k, i);
			upper.rowAdd(k, pivot, -upper(k, i));
		}

		++pivot;
	}
}

template <class T>
void Matrix<T>::print() const {
	for (int32_t i = 0; i < _rows; ++i) {
		for (int32_t j = 0; j < _cols; ++j) {
			std::cout << (*this)(i, j) << "\t";
		}
		std::cout << "\n";
	}
	std::cout << "\n";
}

template <class T>
void Matrix<T>::mirrorRowReduce(Matrix<T>& other) {
	rowReduceLower(&other);

	rowReduceHigher(&other);
}

template <class T>
void Matrix<T>::rowReduceLower(Matrix<T>* other) {
	T element, scale;
	// column the pivot is in
	int32_t pivot = 0;
	for (int32_t i = 0; i < _cols; ++i) {
		bool zero_col = true;
		for (int32_t j = pivot; j < _rows; ++j) {
			// searches down the column for first non-zero entry
			// where non-zero means greater than threshold values
			element = std::fabs((*this)(j, i));
			if (element > _threshold) {
				if (pivot != j) {
					// swap to make the pivot non-zero
					rowSwap(j, pivot);
					if (other != nullptr) {
						other -> rowSwap(j, pivot);
					}
				}
				zero_col = false;
				break;
			} else if (element != 0) {
				// set it to zero if it's not
				(*this)(j, i) = 0;
			}
		}

		if (zero_col) {
			continue;
		}

		// make pivot = 1
		scale = 1.0 / (*this)(pivot, i);
		rowMultiply(pivot, scale);
		if (other != nullptr) {
			other -> rowMultiply(pivot, scale);
		}

		//make all entires under the pivot equal 0
		for (int32_t k = pivot + 1; k < _rows; ++k) {
			scale = -(*this)(k, i);
			rowAdd(k, pivot, scale);
			if (other != nullptr) {
				other -> rowAdd(k, pivot, scale);
			}
		}

		++pivot;
	}
}

template <class T>
void Matrix<T>::rowReduceHigher(Matrix<T>* other) {
	T scale;
	// column to keep track of leading ones
	int32_t temp_col = 0;
	for (int32_t i = 0; i < _rows; ++i) {
		for (int32_t j = temp_col; j < _cols; ++j) {
			// finds leading one of the row
			if (((*this)(i, j) < (1 + _threshold)) &&
				((*this)(i, j) > (1 - _threshold))) {
				temp_col = j;
				break;
			}
		}

		// makes the column above the entry zero
		if (i == 0) { 
			// prevents k from wrapping around if int32_t is unsigned
			continue;
		}
		for (int32_t k = i - 1; k >= 0; --k) {
			scale = -(*this)(k, temp_col);
			rowAdd(k, i, scale);
			if (other != nullptr) {
				other -> rowAdd(k, i, scale);
			}
		}
	}
}

#endif /* _MATRIX_HPP_ */
