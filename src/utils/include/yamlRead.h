#ifndef YAMLREAD_H
#define YAMLREAD_H

#include "../3rdPartLib/yaml-cpp-0.6.2/include/yaml-cpp/yaml.h"
#include "include/common.h"

inline Mat3x3 Mat33FromYaml(string FilePath, string vName)
{
  Mat3x3 ret;
  YAML::Node config = YAML::LoadFile(FilePath);
  const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
  Eigen::Matrix<double,3,3,RowMajor> matRowMajor(vec_double.data());
  ret = matRowMajor;
  return ret;
}
inline Mat4x4 Mat44FromYaml(string FilePath, string vName)
{
  Mat4x4 ret;
  YAML::Node config = YAML::LoadFile(FilePath);
  const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
  Eigen::Matrix<double,4,4,RowMajor> matRowMajor(vec_double.data());
  ret = matRowMajor;
  return ret;
}
inline double getDoubleVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const double ret = config[vName].as<double>();
  return ret;
}
inline int getIntVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const int ret = config[vName].as<int>();
  return ret;
}
inline bool getBoolVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const bool ret = config[vName].as<bool>();
  return ret;
}
inline string getStringFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const string ret = config[vName].as<string>();
  return ret;
}
#endif // YAML_EIGEN_H
