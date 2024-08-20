#include <optiminf/util/file_system_util.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace optiminf
{
std::string FileSystemUtil::getDirectoryOfFile(const std::string& file_path)
{
  // Inspired by https://stackoverflow.com/questions/8518743/get-directory-from-file-path-c
  std::string directory_path;

  const size_t path_end_index = file_path.rfind('/');
  if (path_end_index != std::string::npos)
  {
    directory_path = file_path.substr(0, path_end_index);
  }

  return directory_path;
}

bool FileSystemUtil::isDirectoryExistent(const std::string& directory_to_check)
{
  // Inspired by https://stackoverflow.com/questions/18100097/portable-way-to-check-if-directory-exists-windows-linux-c)
  struct stat file_info;

  return directory_to_check.size() > 0 && (stat(directory_to_check.c_str(), &file_info) == 0 && (file_info.st_mode & S_IFDIR) != 0);
}

bool FileSystemUtil::createDirectoryIfNonexistent(const std::string& directory_to_create)
{
  int success = -1;
  if (directory_to_create.size() > 0 && !isDirectoryExistent(directory_to_create))
  {
    const std::string create_dir_command = "mkdir -p " + directory_to_create;
    success = system(create_dir_command.c_str());
  }
  return success == 0;
}

bool FileSystemUtil::checkIfFileExists(const std::string& file_path)
{
  struct stat buffer;
  return (stat(file_path.c_str(), &buffer) == 0);
}

bool FileSystemUtil::deleteFile(const std::string& file_path)
{
  int success = -1;
  if (checkIfFileExists(file_path))
  {
    success = std::remove(file_path.c_str());
  }
  return success == 0;
}

std::string FileSystemUtil::substituteUserHomeDirectory(const std::string& file_path)
{
  std::string ret = file_path;

  if (ret[0] == '~')
  {
    const std::string home = getenv("HOME");
    if (home.size() > 0)
    {
      ret.erase(0, 1);
      ret.insert(0, home);
    }
  }
  return ret;
}

}  // namespace optiminf
