#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <initializer_list>

#define MTXTMP template <typename T, unsigned int rows, unsigned int cols>

/* Matrix class template */
MTXTMP class matrix {
private:
	std::array<std::array<T, cols>, rows> myVal;
	template <typename T_, unsigned int rows_, unsigned int cols_>
		struct calc_det {T_ operator()(const matrix<T_, rows_, cols_>&) const;};
	template <typename T_> struct calc_det<T_, 1, 1> {T_ operator()(const matrix<T_, 1, 1>&) const;};
	template <typename T_> struct calc_det<T_, 2, 2> {T_ operator()(const matrix<T_, 2, 2>&) const;};
	template <typename T_, unsigned int size>
		struct calc_det<T_, size, size> {T_ operator()(const matrix<T_, size, size>&) const;};
	template <typename T_, unsigned int rows_, unsigned int cols_>
		struct calc_inv {const matrix<T_, rows_, cols_> operator()(const matrix<T_, rows_, cols_>&) const;};
	template <typename T_> struct calc_inv<T_, 1, 1> {const matrix<T_, 1, 1> operator()(const matrix<T_, 1, 1>&) const;};
	template <typename T_> struct calc_inv<T_, 2, 2> {const matrix<T_, 2, 2> operator()(const matrix<T_, 2, 2>&) const;};
	template <typename T_, unsigned int size>
		struct calc_inv<T_, size, size> {const matrix<T_, size, size> operator()(const matrix<T_, size, size>&) const;};
	template <typename T_, unsigned int rows_, unsigned int cols_>
		struct calc_uptriag {bool operator()(const matrix<T_, rows_, cols_>&) const;};
	template <typename T_, unsigned int size>
		struct calc_uptriag<T_, size, size> {bool operator()(const matrix<T_, size, size>&) const;};
	template <typename T_, unsigned int rows_, unsigned int cols_>
		struct calc_lwtriag {bool operator()(const matrix<T_, rows_, cols_>&) const;};
	template <typename T_, unsigned int size>
		struct calc_lwtriag<T_, size, size> {bool operator()(const matrix<T_, size, size>&) const;};
public:
	matrix();
	matrix(std::initializer_list<std::initializer_list<T> > list);
	std::array<T, cols> getrow(unsigned int row) const;
	std::array<T, rows> getcol(unsigned int col) const;
	T getval(unsigned int row, unsigned int col) const;
	void setval(unsigned int row, unsigned int col, const T val);
	matrix<T, cols, rows> transpose() const;
	matrix<T, rows, cols> operator+() const;
	matrix<T, rows, cols> operator+(const matrix<T, rows, cols>& operand) const;
	matrix<T, rows, cols>& operator+=(const matrix<T, rows, cols>& operand);
	matrix<T, rows, cols> operator-() const;
	matrix<T, rows, cols> operator-(const matrix<T, rows, cols>& operand) const;
	matrix<T, rows, cols>& operator-=(const matrix<T, rows, cols>& operand);
	matrix<T, rows, cols> operator*(const T operand) const;
	matrix<T, rows, cols>& operator*=(const T operand);
	template <unsigned int opcols>
		matrix<T, rows, opcols> operator*(const matrix<T, cols, opcols>& operand) const;
	T determinant() const;
	matrix<T, rows, cols> invert() const;
	template <typename T_, unsigned int rows_, unsigned int cols_>
		bool operator==(const matrix<T_, rows_, cols_>& operand) const;
	template <typename T_, unsigned int rows_, unsigned int cols_>
		bool operator!=(const matrix<T_, rows_, cols_>& operand) const;
	bool is_upper_triangular() const;
	bool is_lower_triangular() const;
	bool is_triangular() const;
	bool is_diagonal() const;
};

/* Identity matrix */
template <typename T, unsigned size> matrix<T, size, size> identity() {
	static_assert(size > 0, "Invalid size of matrix");
	matrix<T, size, size> e;
	for (unsigned i = 0; i < size; i++)
		for (unsigned j = 0; j < size; j++)
			e.setval(i, j, (i == j) ? 1 : 0);
	return e;
}

/* Default constructor */
MTXTMP matrix<T, rows, cols>::matrix() {
	static_assert((rows * cols) > 0, "Invalid size of matrix");
	return;
}

/* Constructor with initializer list */
MTXTMP matrix<T, rows, cols>::matrix(std::initializer_list<std::initializer_list<T> > list) {
	static_assert((rows * cols) > 0, "Invalid size of matrix");
	unsigned int row = 0;
	for (auto k = list.begin(); k != list.end(); ++k) {
		unsigned int col = 0;
		for (auto l = k->begin(); l != k->end(); ++l) {
			myVal[row][col++] = *l;
		}
		++row;
	}
	return;
}

/* Row getter */
MTXTMP std::array<T, cols> matrix<T, rows, cols>::getrow(unsigned int row) const {
	return std::array<T, cols>(myVal[row]);
}

/* Column getter */
MTXTMP std::array<T, rows> matrix<T, rows, cols>::getcol(unsigned int col) const {
	std::array<T, rows> ans;
	for (unsigned int i = 0; i < cols; i++)
		ans[i] = myVal[i][col];
	return ans;
}

/* Getter */
MTXTMP T matrix<T, rows, cols>::getval(unsigned int row, unsigned int col) const {
	return myVal[row][col];
}

/* Setter */
MTXTMP void matrix<T, rows, cols>::setval(unsigned int row, unsigned int col, const T val) {
	myVal[row][col] = val;
	return;
}

/* Transpose */
MTXTMP matrix<T, cols, rows> matrix<T, rows, cols>::transpose() const {
	matrix<T, cols, rows> ans;
	for (unsigned int i = 0; i < cols; i++)
		for (unsigned int j = 0; j < rows; j++)
			ans[i][j] = myVal[j][i];
	return ans;
}

/* Unary plus */
MTXTMP matrix<T, rows, cols> matrix<T, rows, cols>::operator+() const {
	return *this;
}

/* Negation */
MTXTMP matrix<T, rows, cols> matrix<T, rows, cols>::operator-() const {
	return (*this) * ((T)(-1));
}

/* Addition */
MTXTMP matrix<T, rows, cols> matrix<T, rows, cols>::operator+(const matrix<T, rows, cols>& operand) const {
	matrix<T, rows, cols> ans;
	for (unsigned int i = 0; i < rows; i++)
		for (unsigned int j = 0; j < cols; j++)
			ans.myVal[i][j] = myVal[i][j] + operand.myVal[i][j];
	return ans;
}
MTXTMP matrix<T, rows, cols>& matrix<T, rows, cols>::operator+=(const matrix<T, rows, cols>& operand) {
	for (unsigned int i = 0; i < rows; i++)
		for (unsigned int j = 0; j < cols; j++)
			myVal[i][j] += operand.myVal[i][j];
	return *this;
}

/* Subtraction */
MTXTMP matrix<T, rows, cols> matrix<T, rows, cols>::operator-(const matrix<T, rows, cols>& operand) const {
	return (*this) + (-operand);
}
MTXTMP matrix<T, rows, cols>& matrix<T, rows, cols>::operator-=(const matrix<T, rows, cols>& operand) {
	for (unsigned int i = 0; i < rows; i++)
		for (unsigned int j = 0; j < cols; j++)
			myVal[i][j] -= operand.myVal[i][j];
	return *this;
}

/* Scalar multiplication */
MTXTMP matrix<T, rows, cols> matrix<T, rows, cols>::operator*(const T operand) const {
	matrix<T, rows, cols> ans;
	for (unsigned int i = 0; i < rows; i++)
		for (unsigned int j = 0; j < cols; j++)
			ans.myVal[i][j] = myVal[i][j] * operand;
	return ans;
}
MTXTMP matrix<T, rows, cols>& matrix<T, rows, cols>::operator*=(const T operand) {
	for (unsigned int i = 0; i < rows; i++)
		for (unsigned int j = 0; j < cols; j++)
			myVal[i][j] *= operand;
	return *this;
}

/* Matrix multiplication */
MTXTMP template <unsigned int opcols> matrix<T, rows, opcols> matrix<T, rows, cols>::operator*(const matrix<T, cols, opcols>& operand) const {
	matrix<T, rows, opcols> ans = {};
	for (unsigned int i = 0; i < rows; i++)
		for (unsigned int j = 0; j < opcols; j++)
			for (unsigned int k = 0; k < cols; k++)
				ans.setval(i, j, ans.getval(i, j) + this->getval(i, k) * operand.getval(k, j));
	return ans;
}

/* Determinant (function object, error) */
MTXTMP template <typename T_, unsigned int rows_, unsigned int cols_>
	T_ matrix<T, rows, cols>::calc_det<T_, rows_, cols_>::operator()(const matrix<T_, rows_, cols_>&) const {
		throw std::domain_error("Not a square matrix");
}

/* 1x1 matrix (in fact this is a scalar) determinant */
MTXTMP template <typename T_> T_ matrix<T, rows, cols>::calc_det<T_, 1, 1>::operator()(const matrix<T_, 1, 1>& arg) const {
	return arg.getval(0, 0);
}

/* 2x2 matrix determinant */
MTXTMP template <typename T_> T_ matrix<T, rows, cols>::calc_det<T_, 2, 2>::operator()(const matrix<T_, 2, 2>& arg) const {
	return arg.getval(0, 0) * arg.getval(1, 1) - arg.getval(0, 1) * arg.getval(1, 0);
}

/* nxn matrix determinant */
MTXTMP template <typename T_, unsigned int size> T_ matrix<T, rows, cols>::calc_det<T_, size, size>::operator()(const matrix<T_, size, size>& arg) const {
	T_ det;
	if (arg.is_triangular()) {
		det = T_(1);
		for (int i = 0; i < size; i++)
			det *= arg.getval(i, i);
	} else {
		matrix<T_, size - 1, size - 1> minor;
		det = T_(0);
		for (int i = 0; i < size; i++) {
			for (int p = 0; p < (size - 1); p++)
				for (int q = 1; q < size; q++)
					minor.setval(p, q - 1, arg.getval((p >= i) ? (p + 1) : p, q));
			if (i % 2 == 0)
				det += arg.getval(i, 0) * minor.determinant();
			else
				det -= arg.getval(i, 0) * minor.determinant();
		}
	}
	return det;
}

/* Determinant */
MTXTMP T matrix<T, rows, cols>::determinant() const {
	return calc_det<T, rows, cols>()(*this);
}

/* Invertion (function object, error) */
MTXTMP template <typename T_, unsigned int rows_, unsigned int cols_> const matrix<T_, rows_, cols_>
	matrix<T, rows, cols>::calc_inv<T_, rows_, cols_>::operator()(const matrix<T_, rows_, cols_>&) const {
		throw std::domain_error("Not a square matrix");
}

/* 1x1 matrix (in fact this is a scalar) inversion */
MTXTMP template <typename T_> const matrix<T_, 1, 1> matrix<T, rows, cols>::calc_inv<T_, 1, 1>::operator()(const matrix<T_, 1, 1>& arg) const {
	T_ det = arg.determinant();
	if (det == T_(0)) throw std::domain_error("Cannot invert a singular matrix");
	return matrix<T_, 1, 1>({
		{arg.getval(0, 0) / det,},
	});
}

/* 2x2 matrix inversion */
MTXTMP template <typename T_> const matrix<T_, 2, 2> matrix<T, rows, cols>::calc_inv<T_, 2, 2>::operator()(const matrix<T_, 2, 2>& arg) const {
	T_ det = arg.determinant();
	if (det == T_(0)) throw std::domain_error("Cannot invert a singular matrix");
	return matrix<T_, 2, 2>({
		{arg.getval(1, 1) / det, -arg.getval(0, 1) / det,},
		{-arg.getval(0, 1) / det, arg.getval(0, 0) / det,},
	});
}

/* nxn matrix inversion */
MTXTMP template <typename T_, unsigned int size> const matrix<T_, size, size>
	matrix<T, rows, cols>::calc_inv<T_, size, size>::operator()(const matrix<T_, size, size>& arg) const {
		matrix<T_, size, size> inverse;
		matrix<T_, size - 1, size - 1> minor;
		T_ det = arg.determinant();
		if (det == T_(0)) throw std::domain_error("Cannot invert a singular matrix");
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				for (int p = 0; p < (size - 1); p++)
					for (int q = 0; q < (size - 1); q++)
						minor.setval(p, q, arg.getval((p >= i) ? (p + 1) : p, (q >= j) ? (q + 1) : q));
				if ((i + j) % 2 == 0)
					inverse.setval(j, i, minor.determinant() / det);
				else
					inverse.setval(j, i, -minor.determinant() / det);
			}
		}
		return inverse;
}

/* Inversion */
MTXTMP matrix<T, rows, cols> matrix<T, rows, cols>::invert() const {
	return calc_inv<T, rows, cols>()(*this);
}

/* Equality */
MTXTMP template <typename T_, unsigned int rows_, unsigned int cols_>
	bool matrix<T, rows, cols>::operator==(const matrix<T_, rows_, cols_>& operand) const {
		if ((rows != rows_) || (cols != cols_))
			return false;
		for (unsigned i = 0; i < rows; i++)
			for (unsigned j = 0; j < cols; j++)
				if (myVal[i][j] != operand.getval(i, j))
					return false;
		return true;
}

/* Non-equality */
MTXTMP template <typename T_, unsigned int rows_, unsigned int cols_>
	bool matrix<T, rows, cols>::operator!=(const matrix<T_, rows_, cols_>& operand) const {
		return !((*this) == operand);
}

/* is upper triangular? (function object, error) */
MTXTMP template <typename T_, unsigned int rows_, unsigned int cols_>
	bool matrix<T, rows, cols>::calc_uptriag<T_, rows_, cols_>::operator()(const matrix<T_, rows_, cols_>&) const {
		throw std::domain_error("Not a square matrix");
}

/* is upper triangular? (function object, square) */
MTXTMP template <typename T_, unsigned int size> bool matrix<T, rows, cols>::calc_uptriag<T_, size, size>::operator()(const matrix<T_, size, size>& arg) const {
	for (int i = 0; i < size; i++)
		for (int j = 0; j < i; j++)
			if (arg.getval(i, j) != T_(0))
				return false;
	return true;
}

/* is lower triangular? (function object, error) */
MTXTMP template <typename T_, unsigned int rows_, unsigned int cols_>
	bool matrix<T, rows, cols>::calc_lwtriag<T_, rows_, cols_>::operator()(const matrix<T_, rows_, cols_>&) const {
		throw std::domain_error("Not a square matrix");
}

/* is lower triangular? (function object, square) */
MTXTMP template <typename T_, unsigned int size> bool matrix<T, rows, cols>::calc_lwtriag<T_, size, size>::operator()(const matrix<T_, size, size>& arg) const {
	for (int i = 0; i < size; i++)
		for (int j = i + 1; j < size; j++)
			if (arg.getval(i, j) != T_(0))
				return false;
	return true;
}

/* is upper triangular? */
MTXTMP bool matrix<T, rows, cols>::is_upper_triangular() const {
	return calc_uptriag<T, rows, cols>()(*this);
}

/* is lower triangular? */
MTXTMP bool matrix<T, rows, cols>::is_lower_triangular() const {
	return calc_lwtriag<T, rows, cols>()(*this);
}

/* is triangular? */
MTXTMP bool matrix<T, rows, cols>::is_triangular() const {
	return is_upper_triangular() || is_lower_triangular();
}

/* is diagonal? */
MTXTMP bool matrix<T, rows, cols>::is_diagonal() const {
	return is_upper_triangular() && is_lower_triangular();
}

#endif
