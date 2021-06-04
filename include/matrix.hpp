#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <cstdlib>
#include <vector>
#include <assert.h>
#include <algorithm>

///////////////////////////////////////
// struct MatrixElement
// elements of Matrix class, represent non-zero entry in a matrix
struct MatrixElement
{
    int row, col;	// row and column of the element
    double value;	// value of the element

    // constructor
    MatrixElement(int r, int c, double v)
            : row(r), col(c), value(v) {}

    // compare function for Matrix::SortMatrix() function
    static bool order (MatrixElement e1, MatrixElement e2)
    {
        if (e1.row < e2.row) return true;
        if (e1.row == e2.row) return (e1.col < e2.col);
        return false;
    }
};

// class declaration
typedef std::vector<MatrixElement> MatrixElementList;

// class Matrix definition
class Matrix
{
private:
    int m, n;	// size of matrix (m = # of rows, n # of columns)
    MatrixElementList elements;	// list of non-zero entries
    int * rowIndex;				// row indice of non-zero entries

    // fields for CG method
    double* diagInv;
    double* r;
    double* r2;
    double* d;
    double* d2;
    double* q;
    double* s;
    double* s2;

public:
    // constructor & destructor
    Matrix(int m, int n) : m(m), n(n)
    {
        rowIndex = new int[m+1];
        diagInv = new double[m];
        r = new double[m];
        r2 = new double[m];
        d = new double[m];
        d2 = new double[m];
        q = new double[m];
        s = new double[m];
        s2 = new double[m];
    }
    ~Matrix()
    {
        delete[] rowIndex;
        delete[] r;
        delete[] r2;
        delete[] d;
        delete[] d2;
        delete[] q;
        delete[] s;
        delete[] s2;
    }

    /////////////////////////////////////
    // function AddElement
    // add a new entry into the matrix
    void AddElement(int row, int col, double value)
    {
        elements.push_back(MatrixElement(row, col, value));
    }

    /////////////////////////////////////
    // function SortMatrix
    // sort the matrix elements after you add ALL elements into the matrix
    void SortMatrix()
    {
        sort(elements.begin( ), elements.end( ), MatrixElement::order);

        for (int i=0; i<m+1; i++)
            rowIndex[i] = 0;
        for (int i=0; i<(int)elements.size(); i++)
            rowIndex[elements[i].row + 1] = i + 1;

        for (int i=0; i<m; i++)
            diagInv[i] = 0;
        for (int i=0; i<(int)elements.size(); i++)
            if (elements[i].row == elements[i].col)
                diagInv[elements[i].row] = 1.0 / elements[i].value;
    }


    /////////////////////////////////////
    // function Multiply
    // compute A * xIn = xOut
    // the arrays pointed by xIn and xOut have to be pre-allocated
    // and have enough space
    void Multiply(double* xIn, double* xOut)
    {
        for (int i=0; i<m; i++)
        {
            double sum = 0;
            for (int j=rowIndex[i]; j<rowIndex[i+1]; j++)
                sum += elements[j].value * xIn[elements[j].col];
            xOut[i] = sum;
        }
    }
    /////////////////////////////////////
    // Multiply PreMultiply
    // compute xIn * A = xOut
    // the arrays pointed by xIn and xOut have to be pre-allocated
    // and have enough space
    void PreMultiply(double* xIn, double* xOut)
    {
        for (int i=0; i<n; i++) xOut[i] = 0;

        for (int i=0; i<m; i++)
        {
            for (int j=rowIndex[i]; j<rowIndex[i+1]; j++)
                xOut[elements[j].col] += elements[j].value * xIn[i];
        }
    }

    /**********************************************/
    /* function: BCG                              */
    /* description: solve Ax = b for unknowns x   */
    /**********************************************/
    void BCG(double* b, double* x)
    {
        assert(m == n);

        auto mse = [=](double *vec1, double *vec2){
            double sum = 0;
            for(int i = 0; i < m; i++){
                sum += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
            }
            return 1.0 * sum / m;
        };

        /* Initialization */
        for(int i = 0; i < m; i++){
            x[i] = b[i];
        }

        /* d = r = b - Ax */
        double Ax[m];
        this->Multiply(x, Ax);
        for(int i = 0; i < m; i++){
            r[i] = b[i] - Ax[i];
            d[i] = r[i];
        }

        for(int iter = 0; iter < 100; iter++){
            /* a(i) = r(i)^T * r(i) / (d(i)^T * A * d(i)) */
            double Ad[m];
            this->Multiply(d, Ad);
            double numerator = 0, denominator = 0;
            for(int i = 0; i < m; i++){
                numerator += r[i] * r[i];
                denominator += d[i] * Ad[i];
            }
            double alpha = 1.0 * numerator / denominator;

            /* x(i+1) = x(i) + a(i)*d(i) */
            for(int i = 0; i < m; i++){
                x[i] += alpha * d[i];
            }

            /* r(i+1) = r(i) - a(i)*A*d(i) */
            for(int i = 0; i < m; i++){
                r2[i] = r[i] - alpha * Ad[i];
            }

            /* beta(i+1) = r(i+1)^T * r(i+1) / (r(i)^T * r(i)) */
            numerator = 0;
            denominator = 0;
            for(int i = 0; i < m; i++){
                numerator += r2[i] * r2[i];
                denominator += r[i] * r[i];
            }
            double beta = 1.0 * numerator / denominator;

            /* d(i+1) = r(i+1) + beta(i+1) * d(i) */
            for(int i = 0; i < m; i++){
                d2[i] = r2[i] + beta * d[i];
            }

            if(mse(d, d2) < 1e-10){
                break;
            }

            for(int i = 0; i < m; i++){
                r[i] = r2[i];
                d[i] = d2[i];
            }

        }
    }

    // friend operators
    friend ostream & operator<< (ostream & out, const Matrix & r)
    {
        for (int i=0; i<r.m; i++)
        {
            for(int j=r.rowIndex[i]; j<r.rowIndex[i+1]; j++)
                out << r.elements[j].value << " ";
            out << endl;
        }

        return out;
    }

    double at(int x, int y){
        return elements[rowIndex[y]+x].value;
    }

};

#endif