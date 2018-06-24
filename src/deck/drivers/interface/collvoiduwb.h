#ifndef __COLLVOID_UWB_H__
#define __COLLVOID_UWB_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define TOKEN_UPDATE M2T(3000)
#define RECEIVE_TIMEOUT M2T(5)
#define REPORT_TIMEOUT M2T(200)
#define ACTIVE_TIMEOUT M2T(2000)
#define CHECK_ACTIVITY_TIMEOUT M2T(500)
#define NUM_ACTIVE_DRONES 3
#define MAX_RESEND 3

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor
#define LPS_TWR_RANGE 0x05 // Report contains all measurement from the anchor
#define LPS_TWR_PASS_TOKEN 0x06 // Report contains all measurement from the anchor
#define LPS_TWR_REPORT_RECEIVED 0x07 // Report contains all measurement from the anchor

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1
#define TAG_MODE 0
#define ANCHOR_MODE 1
#define ANCHOR_PING_COUNT 10
#define TAG_PING_COUNT 0


extern uwbAlgorithm_t collvoidCommAlgorithm;

void send_msg(dwDevice_t *dev, int payload_size);
void resend_msg(dwDevice_t *dev);
void poll_next(dwDevice_t *dev);
void reset_next_poll();
void pass_token(dwDevice_t *dev);

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsTwrTagReportPayload_t;

typedef struct {
    float distance;
    uint8_t placeholder; //don't know why, but the data extraction doesn't work without it
} __attribute__((packed)) lpsTwrRangePayload_t;

typedef struct {
    locoAddress_t dev_address;
    dwTime_t poll_tx;
    dwTime_t poll_rx;
    dwTime_t answer_tx;
    dwTime_t answer_rx;
    dwTime_t final_tx;
    dwTime_t final_rx;
    bool ranging_complete;
    uint32_t last_update;
    int poll_counter;
    int token_resend_counter;
}coll_dist_dev_t;


#endif // __LPS_TWR_TAG_H__