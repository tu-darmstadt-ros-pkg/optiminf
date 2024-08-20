#pragma once

#include <math.h>
#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SparseCore>

#include "optiminf/util/file_system_util.h"
#include "optiminf/error_handling/optiminf_exception.h"

namespace optiminf
{
/**
 * @brief Read only map for a sparse row-major jacobian
 * @tparam Scalar scalar type used for the matrix entries
 */
template <typename Scalar>
using ConstSparseJacobianMap = Eigen::Map<const Eigen::SparseMatrix<Scalar, Eigen::RowMajor>>;

/**
 * @brief Read only map for a dense row-major jacobian
 * @tparam Scalar scalar type used for the matrix entries
 */
template <typename Scalar>
using ConstDenseJacobianMap = Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

/**
 * @brief Dynamic column vector for generic entry types
 * @tparam Scalar scalar type used for the vector entries
 */
template <typename Scalar>
using VectorXS = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

/**
 * @brief Dynamic dense matrix for generic entry types
 * @tparam Scalar scalar type used for the matrix entries
 */
template <typename Scalar>
using MatrixXS = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

/**
 * @brief The Slice struct can be used to represent a specific area of a vector to be accessed.
 * @details This struct is only necessary for Eigen versions prior to 3.4. When using a newer Version of Eigen one can use the new API described here:
 * https://eigen.tuxfamily.org/dox/group__TutorialSlicingIndexing.html
 */
struct Slice
{
  /**
   * @brief Constructor
   * @param start_idx first index to be accessed
   * @param nr_of_elements number of indices to be accesed
   */
  Slice(size_t start_idx, size_t nr_of_elements)
    : start(start_idx)
    , length(nr_of_elements)
  {}

  /**
   *  @brief first index to be accessed
   */
  size_t start;

  /**
   * @brief number of indices to be accesed
   */
  size_t length;

  /**
   * @brief Index after the last index to be accessed
   * @return The index after the last element of the slice
   */
  size_t end() { return start + length; }
};

/**
 * @brief The EigenUtil class contains utility functions for EIGEN based vectors and matrices
 */
class EigenUtil
{
public:
  /**
   * @brief Creates a map that can be used to access and manipulate the given matrix
   * @details makeCompressed is called on the matrix before creating the map
   * @param matrix to be accessed
   * @return Created map
   */
  template <int Options, typename StorageIndex>
  static Eigen::Map<const Eigen::SparseMatrix<double, Options, StorageIndex>> getMapFromMatrix(Eigen::SparseMatrix<double, Options, StorageIndex>& matrix)
  {
    matrix.makeCompressed();
    return Eigen::Map<const Eigen::SparseMatrix<double, Options, StorageIndex>>(matrix.rows(), matrix.cols(), matrix.nonZeros(), matrix.outerIndexPtr(), matrix.innerIndexPtr(),
                                                                                matrix.valuePtr());
  }

  /**
   * @brief Sets all non-zero entries of a sparse matrix to the given value
   * @details This function is usefull e.g to set all non zero values to 1 s.t. you can visualize the sparsity pattern for debugging purposes
   * @param sparse_mat sparse input matrix
   * @param value value to which the non-zeros should be set
   */
  template <typename SparseEigenMatrix>
  static void setAllNonZerosToValue(SparseEigenMatrix& sparse_mat, double value)
  {
    sparse_mat.coeffs() = value;
  }

  /**
   * @brief Checks all entries in the sparse matrix to be finite
   * @details If a value is non finite a NlpErrorStatus with priority level warning is set
   * @param sparse_matrix sparse matrix to be checked
   * @param debug_info_prefix a string that is put at the beginning of the debug message before the information of the position of the non finite value
   */
  template <typename SparseEigenMatrixExpression>
  static void checkSparseMatrixForNonFiniteValue(const SparseEigenMatrixExpression& sparse_matrix, const std::string& debug_info_prefix)
  {
    for (size_t i = 0; i < sparse_matrix.rows(); i++)
    {
      typename SparseEigenMatrixExpression::InnerIterator it(sparse_matrix, i);
      while (it)
      {
        if (!isfinite(it.value()))
        {
          throw NumericError(debug_info_prefix + " Value of matrix is not finite but has value " + std::to_string(it.value()) + " at position (" + std::to_string(it.row()) + ", " +
                             std::to_string(it.col()) + ")");
        }
        ++it;
      }
    }
  }

  /**
   * @brief Checks all entries in the vector to be finite
   * @details If a value is non finite a NlpErrorStatus with priority level warning is set
   * @param vector to be checked
   * @param debug_info_prefix a string that is put at the beginning of the debug message before the information of the position of the non finite value
   */
  template <typename DenseVectorExpression>
  static void checkVectorForNonFiniteValue(const DenseVectorExpression& vector, const std::string& debug_info_prefix)
  {
    for (size_t i = 0; i < vector.rows(); i++)
    {
      double value = vector(i);

      if (!isfinite(value))
      {
        throw NumericError(debug_info_prefix + " Value of vector is not finite but has value " + std::to_string(value) + " in row " + std::to_string(i));
      }
    }
  }

  /**
   * @brief Loads an Eigen matrix from the given CSV file.
   * @param file_path file path to the csv file
   * @return returns the loaded matrix
   */
  template <typename EigenMat>
  static EigenMat loadEigenMatrixFromCsv(const std::string& file_path)
  {
    // Inspired by https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
    std::vector<double> parsed_values;

    std::ifstream file_stream;
    file_stream.open(file_path);

    size_t row_counter = 0;

    std::string parsed_line;
    std::string matrix_entry;

    while (std::getline(file_stream, parsed_line))
    {
      std::stringstream line_stream(parsed_line);

      while (std::getline(line_stream, matrix_entry, ','))
      {
        parsed_values.push_back(std::stod(matrix_entry));
      }
      row_counter++;
    }

    const size_t nr_of_columns = parsed_values.size() / row_counter;
    return EigenMat(Eigen::Map<const Eigen::Matrix<typename EigenMat::Scalar, EigenMat::RowsAtCompileTime, EigenMat::ColsAtCompileTime, Eigen::RowMajor>>(
        parsed_values.data(), row_counter, nr_of_columns));
  }

  /**
   * @brief Saves the Eigen matrix to a given CSV file path.
   * @param file_path file path to the csv file
   * @param matrix matrix to save
   */
  template <typename EigenMat>
  static void saveEigenMatrixAsCsv(EigenMat matrix, std::string file_path)
  {
    FileSystemUtil::createDirectoryIfNonexistent(FileSystemUtil::getDirectoryOfFile(file_path));

    // Inspired by https://aleksandarhaber.com/eigen-matrix-library-c-tutorial-saving-and-loading-data-in-from-a-csv-file/
    std::ofstream file_stream(file_path);
    if (file_stream.is_open())
    {
      // Details regarding the format settings can be found under https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
      const Eigen::IOFormat csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
      file_stream << matrix.format(csv_format);
      file_stream.close();
    }
  }
};

}  // namespace optiminf
