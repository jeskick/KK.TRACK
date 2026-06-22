#include "kk/fw_version.h"

#include "esp_app_desc.h"
#include <string.h>

static char s_tx_ver[32];

const char *kk_fw_local_version(void)
{
    return esp_app_get_description()->version;
}

const char *kk_fw_tx_version(void)
{
    return s_tx_ver[0] != '\0' ? s_tx_ver : "--";
}

void kk_fw_set_tx_version(const char *ver)
{
    if (!ver) {
        s_tx_ver[0] = '\0';
        return;
    }
    strncpy(s_tx_ver, ver, sizeof(s_tx_ver) - 1);
    s_tx_ver[sizeof(s_tx_ver) - 1] = '\0';
}

void kk_fw_clear_tx_version(void)
{
    s_tx_ver[0] = '\0';
}
