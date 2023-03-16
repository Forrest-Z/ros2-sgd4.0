#include "yaml-cpp/yaml.h"

namespace sgd_util
{
    
/**
 * @brief Get the given subnode value.
 * The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
 * including the name of the failed key
 *
 * @tparam T
 * @param node
 * @param key
 * @return T
 */
template <typename T>
T yaml_get_value(const YAML::Node &node, const std::string &key)
{
    try
    {
        return node[key].as<T>();
    }
    catch (YAML::Exception &e)
    {
        std::stringstream ss;
        ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
        throw YAML::Exception(e.mark, ss.str());
    }
}

} // namespace sgd_util


