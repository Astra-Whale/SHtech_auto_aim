#ifndef _CMD_PARSER_HPP_
#define _CMD_PARSER_HPP_

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

class cmd_parser
{
public:
    explicit cmd_parser(const std::string &cfg_path);
    ~cmd_parser() noexcept;

    const std::string &get_string(const std::string &key);
    bool get_bool(const std::string &key);
    void report_unused_keys() noexcept;

private:
    void parse(const std::string &cfg_path);
    void split(char c, const std::string &in, std::vector<std::string> &out);
    void strip(const std::string &c_set, const std::string &in, std::string &out);
    void strip(const std::string &c_set, std::string &in_out);
    void strip(std::string &in_out);
    void rmcmt(std::string &in_out);

    std::map<std::string, std::string> info_;
    std::map<std::string, bool> display_;
    std::set<std::string> all_keys_;
    std::set<std::string> used_keys_;
    bool unused_reported_ = false;
};

#endif