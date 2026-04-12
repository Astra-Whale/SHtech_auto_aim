#include "cmd_parser.hpp"

#include <exception>
#include <sstream>
#include <stdexcept>

cmd_parser::cmd_parser(const std::string &cfg_path)
{
    parse(cfg_path);
}

cmd_parser::~cmd_parser() noexcept
{
    if (std::uncaught_exceptions() > 0)
    {
        return;
    }
    report_unused_keys();
}

const std::string &cmd_parser::get_string(const std::string &key)
{
    auto it = info_.find(key);
    if (it == info_.end())
    {
        throw std::runtime_error("Missing required config key: " + key);
    }
    used_keys_.insert(key);
    return it->second;
}

bool cmd_parser::get_bool(const std::string &key)
{
    auto it = display_.find(key);
    if (it == display_.end())
    {
        throw std::runtime_error("Missing required display key: " + key);
    }
    used_keys_.insert(key);
    return it->second;
}

void cmd_parser::report_unused_keys() noexcept
{
    if (unused_reported_)
    {
        return;
    }

    std::vector<std::string> unused;
    unused.reserve(all_keys_.size());
    for (const auto &key : all_keys_)
    {
        if (used_keys_.find(key) == used_keys_.end())
        {
            unused.push_back(key);
        }
    }

    if (!unused.empty())
    {
        std::ostringstream oss;
        oss << "[WARN] Unused config keys(" << unused.size() << "): ";
        for (size_t i = 0; i < unused.size(); ++i)
        {
            if (i > 0)
            {
                oss << ", ";
            }
            oss << unused[i];
        }
        std::cerr << oss.str() << std::endl;
    }

    unused_reported_ = true;
}

void cmd_parser::split(char c, const std::string &in, std::vector<std::string> &out)
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

void cmd_parser::strip(const std::string &c_set, const std::string &in, std::string &out)
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

void cmd_parser::strip(const std::string &c_set, std::string &in_out)
{
    strip(c_set, in_out, in_out);
}

void cmd_parser::strip(std::string &in_out)
{
    strip(" \t\n", in_out);
}

void cmd_parser::rmcmt(std::string &in_out)
{
    auto l = in_out.find_first_of('#');
    if (l != in_out.npos)
    {
        in_out.erase(in_out.begin() + l, in_out.end());
    }
}
void cmd_parser::parse(const std::string &cfg_path)
{
    info_.clear();
    display_.clear();
    all_keys_.clear();
    used_keys_.clear();
    unused_reported_ = false;

    std::ifstream in(cfg_path);
    std::string line;
    int line_index = 0;
    if (!in.is_open())
    {
        throw std::runtime_error("Open Launch File Fail!");
    }

    while (std::getline(in, line))
    {
        ++line_index;
        rmcmt(line);
        strip(line);
        if (0 == line.length() || '#' == line[0])
        {
            continue;
        }
        else
        {
            std::vector<std::string> p;
            split('=', line, p);
            if (p.size() != 2)
            {
                throw std::runtime_error("invalid cfg format at line " + std::to_string(line_index));
            }
            strip(p[0]);
            strip(p[1]);

            if (p[0].empty() || p[1].empty())
            {
                throw std::runtime_error("empty key/value at line " + std::to_string(line_index));
            }

            if (all_keys_.find(p[0]) != all_keys_.end())
            {
                throw std::runtime_error("duplicate key at line " + std::to_string(line_index) + ": " + p[0]);
            }

            all_keys_.insert(p[0]);

            if (p[1] == "true")
            {
                display_.insert(std::pair<std::string, bool>(p[0], true));
            }
            else if (p[1] == "false")
            {
                display_.insert(std::pair<std::string, bool>(p[0], false));
            }
            else
            {
                info_.insert(std::pair<std::string, std::string>(p[0], p[1]));
            }
        }
    }
}