#ifndef _CMD_PARSER_HPP_
#define _CMD_PARSER_HPP_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace std;

class cmd_parser
{
public:
    cmd_parser() = default;
    void parse(const string &cfg_path, map<string, string> &info, map<string, bool> &display);

private:
    void split(char c, const string &in, vector<string> &out);
    void strip(const string &c_set, const string &in, string &out);
    void strip(const string &c_set, string &in_out);
    void strip(string &in_out);
    void rmcmt(string &in_out);
};

#endif