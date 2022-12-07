/**
 * This file contains the necessary infrastructure to handle linear algebra operations for the Kalman filter
 */

// Utility functions
float** generate_matrix(unsigned m, unsigned n)
{
    // TODO This function allocates memory dynamically and generates an m-by-n matrix
    // and returns a pointer to a pointer pointing to the first element
    
}


// Function definitions

float** transpose_matrix(float** matrix, unsigned m, unsigned n)
{
    // TODO This function allocates memory and fills it with the transpose of m-by-n matrix
    // matrix and returns a double pointer to the first element of the n-by-m transopose
}

float** inverse(float** matrix, unsigned m)
{
    // TODO This function allocates memory and fills it with the inverse of the m-by-m matrix
    // matrix and returns a double pointer to the first element of the new m-by-m matrix
    // If the matrix is singular, print an error message and exit
}

float* solve(float** A, float** B, unsigned m)
{
    // Solve linear system of equations Ax = B, A being of size m-by-m
    // Allocate, fill with solution and return vector x
}

