#ifndef KDL_PARSER_TEST_HELPERS_HPP
#define KDL_PARSER_TEST_HELPERS_HPP

#include "TestConfig.hpp"
#include <fstream>
#include <string>

namespace {
    inline std::string readFile(std::string const& path)
    {
        std::ifstream istream(path);
        return std::string( (std::istreambuf_iterator<char>(istream) ),
                            (std::istreambuf_iterator<char>()    ) );
    }
}

#endif
