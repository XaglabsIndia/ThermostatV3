#include "esp_err.h"

void IRAM_ATTR MultiButtonISR(void *arg);
void MultiButton(void *pvParameter);
void PerformAction(uint32_t duration);
void SetupMultiButton();

extern void HandleOTAMessage( const char* message_id,int DevID,int HubID);
extern int StoredDevID, StoredHubID;