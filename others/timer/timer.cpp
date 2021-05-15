#include "timer.hpp"

using namespace std;
using namespace chrono;

systimer::systimer(void)
{
    time_base = steady_clock::now();
}

systime_t systimer::getTime(void) const
{
    return milliseconds(duration_cast<milliseconds>(steady_clock::now() - time_base)).count();
}

string systimer::getTimeStr(systime_t _t) const
{
    int64_t t = int64_t(_t);
    int64_t ms = t % 1000;
    int64_t s = (t / 1000) % 60;
    int64_t min = (t / 1000 / 60) % 60;
    stringstream ss;
    ss.fill('0');
    ss << setw(2) << min << ":" << setw(2) << s << ":" << setw(3) << ms << ":";
    return ss.str();
}

string systimer::getTimeStr(void) const
{
    return getTimeStr(getTime());
}

const systimer systime;

void timer::tick(void)
{
    begin = systime.getTime();
}

double timer::tock(void)
{
    return systime.getTime() - begin;
}

avg_timer::avg_timer(const std::string &_name, int _up, Log_t _log) : name(_name), up(_up), log(_log) {}

bool avg_timer::tock_avg(void)
{
    time_seq.push_back(tock());
    if (time_seq.size() >= up)
    {
        double time = 0;
        for (auto &i : time_seq)
        {
            time += i;
        }
        time = time / time_seq.size();
        LOGM(log, "%s : %.1f ms", name.c_str(), time);
        time_seq.clear();
        return true;
    }
    return false;
}

fps_counter::fps_counter(const std::string &_name, systime_t _period, Log_t _l) : name(_name), log(_l)
{
    period = _period;
    tick();
}

bool fps_counter::count(void)
{
    if (tock() > period)
    {
        tick();
        LOGM(log, "%s : %d fps", name.c_str(), (int)(c / (period / 1000)));
        c = 0;
        return true;
    }
    c++;
    return false;
}