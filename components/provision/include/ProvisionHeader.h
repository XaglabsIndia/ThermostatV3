#include "esp_err.h"

esp_err_t CheckStoredKeyStatus(char* Key, int* Value);
esp_err_t StartProvision(int TimeInterval, uint8_t Length);
void  ErrorProvisionLight();
