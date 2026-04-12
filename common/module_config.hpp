#ifndef COMMON_MODULE_CONFIG_H
#define COMMON_MODULE_CONFIG_H

namespace pipeline
{

    struct ModuleConfig
    {
        struct DebugOptions
        {
            bool log_text = false;
            bool log_file = false;
            bool show_image = false;
        };
        DebugOptions debug;
    };
}

#endif // COMMON_MODULE_CONFIG_H
