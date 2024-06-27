/*
 * matrix.hpp
 *
 *  Created on: Jun 26, 2024
 *      Author: Louis
 */

#pragma once

#define ARM_MATH_CM7
#include "arm_math.h"

#include <stdexcept>


/*
 * Matrix class.
 * Basically just a C++ wrapper of the
 * ARM Math library (CMSIS-DSP library).
 * It use float number because arm_math.h use float.
 */
template<size_t Rows, size_t Cols>
class Matrix
{
private:
	arm_matrix_instance_f32 m_mat;
    float m_data[Rows * Cols];  // Static array for matrix data

public:
    /*
     * Constructor that initializes the matrix with given data
     */
    Matrix(const float* pInputData)
    {
        m_mat.numRows = Rows;
        m_mat.numCols = Cols;
        m_mat.pData = m_data;

        for (size_t i = 0; i < Rows * Cols; ++i)
        {
        	m_data[i] = pInputData[i];
        }
    }

    /*
     * Constructor with no initialization
     */
    Matrix()
	{
		m_mat.numRows = Rows;
		m_mat.numCols = Cols;
		m_mat.pData = m_data;
	}

    /*
     * Copy constructor
     */
    Matrix(const Matrix& other)
    {
    	m_mat.numRows = other.m_mat.numRows;
    	m_mat.numCols = other.m_mat.numCols;
    	m_mat.pData = m_data;

        for (size_t i = 0; i < other.m_mat.numRows * other.m_mat.numCols; ++i)
        {
        	m_data[i] = other.m_data[i];
        }
    }

    // Delete move constructor and move assignment operator.
    // as static array are used, there is nothing movable.
	//Matrix(Matrix&& other) = delete;
	//Matrix& operator=(Matrix&& other) = delete;



	/*
	 * Addition operator overload
	 */
    template<size_t OtherRows, size_t OtherCols>
    Matrix<Rows, Cols> operator+(const Matrix<OtherRows, OtherCols>& rhs) const
    {
    	static_assert(
    			Rows == OtherRows && Cols == OtherCols,
				"Matrix dimensions must match for addition."
				);

        Matrix<Rows, Cols> result;
        arm_mat_add_f32(&m_mat, &rhs.m_mat, &result.m_mat);

        return result;
    }

    /*
     * Subtraction operator overload
     */
    template<size_t OtherRows, size_t OtherCols>
    Matrix<Rows, Cols> operator-(const Matrix<OtherRows, OtherCols>& rhs) const
    {
    	static_assert(
				Rows == OtherRows && Cols == OtherCols,
				"Matrix dimensions must match for subtraction."
				);

        Matrix<Rows, Cols> result;
        arm_mat_sub_f32(&m_mat, &rhs.m_mat, &result.m_mat);

        return result;
    }

    /*
     * Multiplication operator overload
     */
    template<size_t OtherRows, size_t OtherCols>
    Matrix<Rows, OtherCols> operator*(
    		const Matrix<OtherRows, OtherCols>& rhs
			) const
    {
    	static_assert(
    			Cols == OtherRows,
				"Matrix dimensions must be compatible for multiplication."
				);

        Matrix<Rows, OtherCols> result;
        arm_mat_mult_f32(&m_mat, &rhs.m_mat, &result.m_mat);

        return result;
    }

    /*
     * Addition affectation operator overload
     */
    template<size_t OtherRows, size_t OtherCols>
    Matrix<Rows, Cols>& operator+=(const Matrix<OtherRows, OtherCols>& rhs)
    {
    	static_assert(
				Rows == OtherRows && Cols == OtherCols,
				"Matrix dimensions must match for addition."
				);

        arm_mat_add_f32(&m_mat, &rhs.m_mat, &m_mat);

        return *this;
    }

    /*
     * Subtraction affectation operator overload
     */
    template<size_t OtherRows, size_t OtherCols>
    Matrix<Rows, Cols>& operator-=(const Matrix<OtherRows, OtherCols>& rhs)
    {
    	static_assert(
				Rows == OtherRows && Cols == OtherCols,
				"Matrix dimensions must match for subtraction."
				);

        arm_mat_sub_f32(&m_mat, &rhs.m_mat, &m_mat);

        return *this;
    }

    /*
     * Multiplication affectation operator overload
     */
    template<size_t OtherRows, size_t OtherCols>
    Matrix<Rows, OtherCols> operator*=(const Matrix<OtherRows, OtherCols>& rhs)
	{
    	static_assert(
				Cols == OtherRows,
				"Matrix dimensions must be compatible for multiplication."
				);

		Matrix<Rows, OtherCols> temp;
		arm_mat_mult_f32(&m_mat, &rhs.m_mat, &temp.m_mat);

		*this = temp;

		return *this;
	}

    /*
     * Transpose operation
     */
    Matrix<Cols, Rows> transpose()
    {
        Matrix<Cols, Rows> result; // Swapped Rows and Cols
        arm_mat_trans_f32(&m_mat, &result.m_mat);

        return result;
    }

    /*
     * Utility function for printing matrix values
     */
    void print() const
    {
        /*for (int i = 0; i < mat.numRows; ++i)
        {
            for (int j = 0; j < mat.numCols; ++j)
            {
                std::cout << data[i * mat.numCols + j] << " ";
            }
            std::cout << std::endl;
        }*/
    }
};

