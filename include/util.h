//
// Created by qzj on 2020/6/24.
//

#ifndef RENAMEIMAGE_UTIL_H
#define RENAMEIMAGE_UTIL_H
#include <boost/regex.hpp> //Note: using boost regex instead of C++11 regex as it isn't supported by the compiler until gcc 4.9
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <stdint.h>
#include <string>
#include <stdio.h>

#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
#define MAX_PATH_LEN 256
#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

using namespace std;

void replace_str(std::string& str, const std::string& before, const std::string& after);
int32_t createDirectory(const std::string &directoryPath);
void copy_file(std::string src_, std::string dst_);
int sting2Int(string s);

string getDirEnd(string dataset_dir);
string removeExtension(string filewhole);
void getSortedImages(const boost::filesystem::path &img_dir, std::function<bool(const std::string &)> filter,
                     std::function<bool(const std::string &, const std::string &)> comparator, std::vector<std::string> &img_paths);
void getStereoSortedImages(const string root_path,vector<std::string> &img_l_paths, vector<std::string> &img_r_paths);
void  CheckStereoCali(string root_path, string configYaml);
#endif //RENAMEIMAGE_UTIL_H
