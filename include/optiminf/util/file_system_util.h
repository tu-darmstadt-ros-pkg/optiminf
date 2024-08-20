//--------------------------------------------------------------------------------------------------------------------------------
// Copyright 2024 Felix Biemüller, Technische Universität Darmstadt

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED  TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE
//--------------------------------------------------------------------------------------------------------------------------------

#pragma once

#include <fstream>
#include <iostream>

namespace optiminf
{
/**
 * @brief The FileSystemUtil class defines utility functions for handling file paths, directories and files
 */
class FileSystemUtil
{
public:
  /**
   * @brief Extracts the path to the directory that contains the given file.
   * @param file_path path to the file for which the containing directory should be determined
   * @return returns the directory path with no slash at the end if the path is valid and an empty string in any other case
   */
  static std::string getDirectoryOfFile(const std::string& file_path);

  /**
   * @brief Checks whether a directory exists.
   * @param directory_to_check directory path that should be checked
   * @return ture if the directory exists and false in any other case including when the string is empty
   */
  static bool isDirectoryExistent(const std::string& directory_to_check);

  /**
   * @brief Creates a directory with the specified path if it does not exist.
   * @param directory_to_create the path of the directory that should be created
   * @return true if it has been succesfully created and didn't existed before and false in any other case including when the string is empty
   */
  static bool createDirectoryIfNonexistent(const std::string& directory_to_create);

  /**
   * @brief Checks whether there is a file with the given file path.
   * @param file_path path to the file that should be checked
   * @return true if the file exists
   */
  static bool checkIfFileExists(const std::string& file_path);

  /**
   * @brief Deletes the given file if it exist.
   * @param file_path path to the file that should be deleted
   * @return true if the file existed before and has been successfuly deleted
   */
  static bool deleteFile(const std::string& file_path);

  /**
   * @brief Replace a leading ~ by the current user home directory
   * @param file_path absolute file path that may start with ~ as alias for /home/$USER
   * @return file path where a leading ~ is substituted accordingly
   */
  static std::string substituteUserHomeDirectory(const std::string& file_path);
};
}  // namespace optiminf
