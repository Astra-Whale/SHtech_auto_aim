#include "cmd_parser.hpp"

void cmd_parser::split(char c, const string &in, vector<string> &out)
{
    out.clear();
    if (0 == in.length())
    {
        return;
    }
    unsigned int s = 0, e = 0;
    for (; e < in.length(); e++)
    {
        if (in[e] == c)
        {
            out.push_back(in.substr(s, e - s));
            s = e + 1;
        }
    }
    if (s < in.length())
    {
        out.push_back(in.substr(s, e - s));
    }
}

void cmd_parser::strip(const string &c_set, const string &in, string &out)
{
    int s = 0, e = in.length() - 1;

    while (s < e)
    {
        if (c_set.find(in[s]) != c_set.npos)
        {
            s++;
        }
        else
        {
            break;
        }
    }
    while (e >= s)
    {
        if (c_set.find(in[e]) != c_set.npos)
        {
            e--;
        }
        else
        {
            break;
        }
    }
    out = in.substr(s, e - s + 1);
}

void cmd_parser::strip(const string &c_set, string &in_out)
{
    strip(c_set, in_out, in_out);
}

void cmd_parser::strip(string &in_out)
{
    strip(" \t\n", in_out);
}

void cmd_parser::rmcmt(string &in_out)
{
    auto l = in_out.find_first_of('#');
    if (l != in_out.npos)
    {
        in_out.erase(in_out.begin() + l, in_out.end());
    }
}
void cmd_parser::parse(const string &cfg_path, map<string, string> &info, map<string, bool> &display)
{
    ifstream in(cfg_path);
    string line;
    int line_index = 0;
    if (!in.is_open())
    {
        throw "Open Launch File Fail!";
    }
    while (!in.eof())
    {
        ++line_index;
        getline(in, line);
        rmcmt(line);
        strip(line);
        if (0 == line.length() || '#' == line[0])
        {
            continue;
        }
        else
        {
            vector<string> p;
            split('=', line, p);
            if (p.size() != 2 || p[0].length() == 0 || p[1].length() == 0)
            {
                throw "invalid cfg format at line " + to_string(line_index);
            }
            strip(p[0]);
            strip(p[1]);
            if (p[1] == "true")
            {
                display.insert(pair<string, bool>(p[0], true));
            }
            else if (p[1] == "false")
            {
                display.insert(pair<string, bool>(p[0], false));
            }
            else
            {
                info.insert(pair<string, string>(p[0], p[1]));
            }
        }
    }
}