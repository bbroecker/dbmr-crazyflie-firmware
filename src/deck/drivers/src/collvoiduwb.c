/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_twr_anchor.c: Uwb two way ranging anchor implementation */


#include <string.h>

#include "collvoiduwb.h"

#include "FreeRTOS.h"
#include "task.h"
//#include "debug.h"
#include "log.h"

#include "stabilizer_types.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#include "arm_math.h"
#endif

// Outlier rejection
#ifdef ESTIMATOR_TYPE_kalman
#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
static struct {
  float32_t history[RANGING_HISTORY_LENGTH];
  size_t ptr;
} rangingStats[NUM_ACTIVE_DRONES];
#endif

//#define LOG_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)

// Rangin statistics
static uint32_t rangingPerSec[NUM_ACTIVE_DRONES];
static float rangingSuccessRate[NUM_ACTIVE_DRONES];
// Used to calculate above values
static uint32_t succededRanging[NUM_ACTIVE_DRONES];
static uint32_t failedRanging[NUM_ACTIVE_DRONES];
// Stores timestampes for all collvoid devices
static coll_dist_dev_t coll_dist_dev[NUM_ACTIVE_DRONES];
const locoAddress_t BROADCAST_ADDRESS = 0xffffffffffffffff;


static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int last_dist_dev = 0;
//static int required_dev = -1;


static bool m_have_token = false;
static bool ranging_complete = false;
static bool m_expect_reply = false;
static uint8_t m_resend_counter = 0;
static uint8_t m_next_token_holder_id = 0;
static uint8_t m_next_poll_id = 0;
static uint32_t m_last_update = 0;
static uint32_t m_create_token_counter = 0;
static uint32_t m_current_token_id = 0;
static int m_last_update_dt = 0;
static int m_last_active_check = 0;
static uint8_t m_last_payload_size = 0;


static lpsAlgoOptions_t *options;

float pressure, temperature, asl;
bool pressure_ok;

static uint32_t getDistanceDev(locoAddress_t address) {
    int result = -1;
    for (int i = 0; i < NUM_ACTIVE_DRONES; i++) {
        if (coll_dist_dev[i].dev_address == address) {
            result = i;
            break;
        }
    }
    return result;
}

static void txcallback(dwDevice_t *dev) {
    dwTime_t departure;
    dwGetTransmitTimestamp(dev, &departure);
    departure.full += (options->antennaDelay / 2);
    int dist_dev = getDistanceDev(txPacket.destAddress);
    switch (txPacket.payload[0]) {
        case LPS_TWR_POLL:
//            for (int i = 0; i < NUM_ACTIVE_DRONES; i++) {
            coll_dist_dev[dist_dev].poll_tx = departure;
//            }
            break;
        case LPS_TWR_FINAL:
            coll_dist_dev[dist_dev].final_tx = departure;
            break;
        case LPS_TWR_ANSWER:
            coll_dist_dev[dist_dev].answer_tx = departure;
            break;
    }
}

void update_token_status(dwDevice_t *dev, packet_t *msg) {
    //check if token has been passed
    if (m_have_token && txPacket.payload[LPS_TWR_TYPE] == LPS_TWR_PASS_TOKEN /*&& msg->payload[LPS_TWR_TYPE] == LPS_TWR_POLL*/) {
        m_have_token = false;
        m_expect_reply = false;
//        DEBUG_PRINT("token is passed\r\n");
    }
    if (msg->payload[LPS_TWR_TYPE] == LPS_TWR_POLL && m_have_token && m_current_token_id > msg->token_id){
        m_have_token = false;
        m_current_token_id = msg->token_id;
//        DEBUG_PRINT("I shouldn have token!\r\n");
    }
//    if (m_have_token){
////        DEBUG_PRINT("m_have_token\r\n");
//    }else{
////        DEBUG_PRINT("not m_have_token\r\n");
//    }
}

bool react_to_message(dwDevice_t *dev, packet_t *msg) {
    bool react = true;
    if ((msg->destAddress != options->dw1000Addresses[options->myID])) {
        react = false;
//        DEBUG_PRINT("message is for %d my %d\r\n", (int)getDistanceDev(msg->destAddress), (int)(options->myID));
    }
    if (m_have_token && m_current_token_id < msg->token_id &&
        (msg->payload[LPS_TWR_TYPE] == LPS_TWR_POLL || msg->payload[LPS_TWR_TYPE] == LPS_TWR_REPORT)) {
        react = false;
//        DEBUG_PRINT("My token: %d is better than %d\r\n", (int)m_current_token_id, (int)msg->token_id);

    }
    return react;
}

void initiateRanging(dwDevice_t *dev) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
}

void update_ranging_state(){
    uint32_t dt = 0;
    for (int i = 0; i < NUM_ACTIVE_DRONES; i++) {
        dt = xTaskGetTickCount() - coll_dist_dev[i].last_update;

        if (dt > ACTIVE_TIMEOUT) {
            options->rangingState &= ~(1 << i);
            if (options->failedRanging[i] < options->rangingFailedThreshold) {
                options->failedRanging[i]++;
                options->rangingState |= (1 << i);
            }
            failedRanging[i]++;
        } else {
            options->rangingState |= (1 << i);
            options->failedRanging[i] = 0;
//          DEBUG_PRINT("last_dist_dev %d\r\n", last_dist_dev);
            succededRanging[i]++;
        }
    }
}

static void rxcallback(dwDevice_t *dev) {
//    DEBUG_PRINT("rxcallback\r\n");

    dwTime_t arival = {.full=0};
    int dataLength = dwGetDataLength(dev);

//    if (dataLength == 0) return 0;
    if (dataLength == 0) return;
    m_last_update = xTaskGetTickCount();
    packet_t rxPacket;
    memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

    dwGetData(dev, (uint8_t * ) & rxPacket, dataLength);
    //if device == -1, doesn't have the device address
    int current_dev = getDistanceDev(rxPacket.sourceAddress);
    update_token_status(dev, &rxPacket);
    if (!react_to_message(dev, &rxPacket)) {
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
        return;
    }

//        DEBUG_PRINT("rxcallback for me!!! \r\n");

    txPacket.destAddress = rxPacket.sourceAddress;
    txPacket.sourceAddress = options->dw1000Addresses[options->myID];
    m_expect_reply = true;
    switch (rxPacket.payload[LPS_TWR_TYPE]) {
        case LPS_TWR_PASS_TOKEN:
//            DEBUG_PRINT("LPS_TWR_PASS_TOKEN\r\n");
//            DEBUG_PRINT("LPS_TWR_PASS_TOKEN myID: %d other %d \r\n", options->myID, current_dev);
            txPacket.destAddress = options->dw1000Addresses[options->myID + 1];
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;
//            coll_dist_dev[options->myID + 1].poll_counter = 0;
//            m_next_token_holder_id = (options->myID + 1) % (NUM_ACTIVE_DRONES - 1);
            m_next_token_holder_id = options->myID;
            m_next_poll_id = options->myID + 2;
            m_have_token = true;
            m_current_token_id = rxPacket.token_id;
            send_msg(dev, 0);
//            initiateRanging(dev);
            break;
        case LPS_TWR_POLL:
//            DEBUG_PRINT("LPS_TWR_POLL\r\n");
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
            txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
            dwGetReceiveTimestamp(dev, &arival);
            arival.full -= (options->antennaDelay / 2);
            coll_dist_dev[current_dev].poll_rx = arival;
            m_current_token_id = rxPacket.token_id;
//            DEBUG_PRINT("LPS_TWR_POLL myID: %d other %d token %d\r\n", options->myID, current_dev, (int)m_current_token_id);
            send_msg(dev, 0);
//            DEBUG_PRINT("ANCHOR_MODE \r\n");
            break;

        case LPS_TWR_ANSWER:
//            DEBUG_PRINT("LPS_TWR_ANSWER\r\n");
//            DEBUG_PRINT("LPS_TWR_ANSWER myID: %d other %d \r\n", options->myID, current_dev);

            if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
//                DEBUG_PRINT("LPS_TWR_ANSWER NOT SEQ!!! rx: %d currseq %d\r\n",rxPacket.payload[LPS_TWR_SEQ], curr_seq);
                //return 0;
            }
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
            txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
            dwGetReceiveTimestamp(dev, &arival);
            arival.full -= (options->antennaDelay / 2);
            coll_dist_dev[current_dev].answer_rx = arival;
            send_msg(dev, 0);
            break;
        case LPS_TWR_FINAL:
        {
//            DEBUG_PRINT("LPS_TWR_FINAL myID: %d other %d \r\n", options->myID, current_dev);

// DEBUG_PRINT("LPS_TWR_FINAL\r\n");
            if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
//                DEBUG_PRINT("LPS_TWR_FINAL NOT SEQ!!! rx: %d currseq %d\r\n",rxPacket.payload[LPS_TWR_SEQ], curr_seq);
                //return 0;
            }
            lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t * )(txPacket.payload + 2);
//             DEBUG_PRINT("FINAL\r\n");
            dwGetReceiveTimestamp(dev, &arival);
            arival.full -= (options->antennaDelay / 2);
            coll_dist_dev[current_dev].final_rx = arival;

            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
            txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
            memcpy(&report->pollRx, &coll_dist_dev[current_dev].poll_rx, 5);
            memcpy(&report->answerTx, &coll_dist_dev[current_dev].answer_tx, 5);
            memcpy(&report->finalRx, &coll_dist_dev[current_dev].final_rx, 5);
            report->pressure = pressure;
            report->temperature = temperature;
            report->asl = asl;
            report->pressure_ok = pressure_ok;
            send_msg(dev, sizeof(lpsTwrTagReportPayload_t));
            break;
        }
        case LPS_TWR_REPORT:
        {
//            DEBUG_PRINT("LPS_TWR_REPORT myID: %d other %d \r\n", options->myID, current_dev);
//            DEBUG_PRINT("LPS_TWR_REPORT\r\n");
            lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t * )(rxPacket.payload + 2);
            double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

            if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
//                DEBUG_PRINT("LPS_TWR_REPORT NOT SEQ!!! rx: %d currseq %d\r\n",rxPacket.payload[LPS_TWR_SEQ], curr_seq);
                //return 0;
            }
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_RANGE;
            txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
            memcpy(&coll_dist_dev[current_dev].poll_rx, &report->pollRx, 5);
            memcpy(&coll_dist_dev[current_dev].answer_tx, &report->answerTx, 5);
            memcpy(&coll_dist_dev[current_dev].final_rx, &report->finalRx, 5);

            tround1 = coll_dist_dev[current_dev].answer_rx.low32 - coll_dist_dev[current_dev].poll_tx.low32;
            treply1 = coll_dist_dev[current_dev].answer_tx.low32 - coll_dist_dev[current_dev].poll_rx.low32;
            tround2 = coll_dist_dev[current_dev].final_rx.low32 - coll_dist_dev[current_dev].answer_tx.low32;
            treply2 = coll_dist_dev[current_dev].final_tx.low32 - coll_dist_dev[current_dev].answer_rx.low32;

            tprop_ctn = ((tround1 * tround2) - (treply1 * treply2)) / (tround1 + tround2 + treply1 + treply2);

            tprop = tprop_ctn / LOCODECK_TS_FREQ;
            options->distance[current_dev] = SPEED_OF_LIGHT * tprop;
            //DEBUG_PRINT("CURRENT_DEV %d REPORT distance %f\r\n",current_dev, (double)options->distance[current_dev]);
            options->pressures[current_dev] = report->asl;

            #ifdef ESTIMATOR_TYPE_kalman
            // Outliers rejection
            rangingStats[current_dev].ptr = (rangingStats[current_dev].ptr + 1) % RANGING_HISTORY_LENGTH;
            float32_t mean;
            float32_t stddev;

            arm_std_f32(rangingStats[current_dev].history, RANGING_HISTORY_LENGTH, &stddev);
            arm_mean_f32(rangingStats[current_dev].history, RANGING_HISTORY_LENGTH, &mean);
            float32_t diff = fabsf(mean - options->distance[last_dist_dev]);

            rangingStats[last_dist_dev].history[rangingStats[last_dist_dev].ptr] = options->distance[current_dev];

            if (options->anchorPositionOk && (diff < (OUTLIER_TH*stddev))) {
              distanceMeasurement_t dist;
              dist.distance = options->distance[current_dev];
              dist.x = options->anchorPosition[current_dev].x;
              dist.y = options->anchorPosition[current_dev].y;
              dist.z = options->anchorPosition[current_dev].z;
              dist.stdDev = 0.25;
              stateEstimatorEnqueueDistance(&dist);
            }
            #endif
            coll_dist_dev[current_dev].last_update = m_last_update;
//            ranging_complete = false;
            m_expect_reply = true;
            last_dist_dev = current_dev;
            lpsTwrRangePayload_t *range_report = (lpsTwrRangePayload_t * )(txPacket.payload + 2);
            range_report->distance = (float)options->distance[current_dev];
//            range_report->placeholder = 16;
//            DEBUG_PRINT("LPS_TWR_REPORT %d distance %f\r\n", curr_seq, (double)range_report->distance);
            send_msg(dev, sizeof(lpsTwrRangePayload_t));
//            return 0;
//            return 0;
            break;
        }
        case LPS_TWR_RANGE:
        {
//            DEBUG_PRINT("LPS_TWR_RANGE myID: %d other %d \r\n", options->myID, current_dev);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT_RECEIVED;
            txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
            lpsTwrRangePayload_t *range_report = (lpsTwrRangePayload_t * )(rxPacket.payload + 2);
            options->distance[current_dev] = (float)range_report->distance;
//            DEBUG_PRINT("LPS_TWR_RANGE %f datalen %d\r\n", (double)range_report->distance, dataLength);
            m_expect_reply = false;
            ranging_complete = true;
            send_msg(dev, 0);
            coll_dist_dev[current_dev].last_update = m_last_update;
            break;
        }
        case LPS_TWR_REPORT_RECEIVED:
        {
//            DEBUG_PRINT("LPS_TWR_REPORT_RECEIVED myID: %d other %d \r\n", options->myID, current_dev);
            m_expect_reply = false;
            ranging_complete = true;
            initiateRanging(dev);
//            DEBUG_PRINT("LPS_TWR_REPORT_RECEIVED %d\r\n", curr_seq);
        }

    }

    last_dist_dev = current_dev;
//    return MAX_TIMEOUT;
}

void resend_msg(dwDevice_t *dev) {
    m_last_update = xTaskGetTickCount();
    if (txPacket.payload[LPS_TWR_TYPE] == LPS_TWR_POLL){
        int dist = getDistanceDev(txPacket.destAddress);
        coll_dist_dev[dist].poll_counter++;
//        DEBUG_PRINT("resend_msg rec: %d type: %d my_id: %d\r\n",(int)dist ,(int)txPacket.payload[LPS_TWR_TYPE], options->myID);

    } else if (txPacket.payload[LPS_TWR_TYPE] == LPS_TWR_PASS_TOKEN){
        int dist = getDistanceDev(txPacket.destAddress);
        coll_dist_dev[dist].token_resend_counter++;
    }
    m_resend_counter++;
    send_msg(dev, m_last_payload_size);
}

void send_msg(dwDevice_t *dev, int payload_size) {
//    if (m_resend_counter <= 0 || true) {
//        int dist = getDistanceDev(txPacket.destAddress);
//        DEBUG_PRINT("SEND rec: %d type: %d counter: %d\r\n", (int) dist, (int) txPacket.payload[LPS_TWR_TYPE],
//                    coll_dist_dev[dist].poll_counter);
//    }
//    DEBUG_PRINT("send_msg\r\n");
//    dwIdle(dev);

    txPacket.token_id = m_current_token_id;
    m_last_payload_size = payload_size;
    dwNewTransmit(dev);
    dwSetDefaults(dev);
    dwSetData(dev, (uint8_t * ) & txPacket, MAC802154_HEADER_LENGTH + 2 + payload_size);

    dwWaitForResponse(dev, true);
    dwStartTransmit(dev);
}

void poll_next(dwDevice_t *dev) {
//    DEBUG_PRINT("poll_next\r\n");
    m_resend_counter = 0;
    m_expect_reply = true;
    curr_seq++;
//    DEBUG_PRINT("POLL_NEXT %d\r\n", m_next_poll_id);
    dwIdle(dev);
    txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
    txPacket.payload[LPS_TWR_SEQ] = curr_seq;
    txPacket.sourceAddress = options->dw1000Addresses[options->myID];
    txPacket.destAddress = options->dw1000Addresses[m_next_poll_id];
    coll_dist_dev[m_next_poll_id].poll_counter = 0;
    last_dist_dev = m_next_poll_id;
//    txPacket.token_id = m_create_token_counter * NUM_ACTIVE_DRONES + options->myID;
    txPacket.token_id = options->myID;
    m_current_token_id = txPacket.token_id;
    do {
        m_next_poll_id++;
    } while (m_next_poll_id == options->myID);
    m_last_update = xTaskGetTickCount();
    send_msg(dev, 0);

}



static uint32_t collvoidCommOnEvent(dwDevice_t *dev, uwbEvent_t event) {
    static uint32_t statisticStartTick = 0;
//    DEBUG_PRINT("collvoidCommOnEvent \r\n");

    if (statisticStartTick == 0) {
        statisticStartTick = xTaskGetTickCount();
    }
    int32_t t_out = 20;
//    int32_t t_out = 10;
    switch (event) {
        case eventPacketReceived:
            rxcallback(dev);
//            return MAX_TIMEOUT;
            return t_out;
//            break;
        case eventPacketSent:
            txcallback(dev);
//            return 10;
            return t_out;
//            break;
        case eventTimeout:  // Comes back to timeout after each ranging attempt


            ranging_complete = false;
            initiateRanging(dev);
//            return 10;
            return t_out;
//            break;
        case eventReceiveTimeout:
        case eventReceiveFailed:
            initiateRanging(dev);
            return t_out;
        case eventTokenHandling:
            //This is an empty statement.
//            m_have_token = true; //remove later
            m_last_update_dt = (xTaskGetTickCount() - m_last_update);
//            DEBUG_PRINT("eventTokenHandling %d\r\n", m_last_update_dt);
//            if (m_last_update_dt > TOKEN_UPDATE) {
            if ((xTaskGetTickCount() - m_last_active_check) > CHECK_ACTIVITY_TIMEOUT){
                m_last_active_check = xTaskGetTickCount();
                update_ranging_state();
            }
            if (m_last_update_dt > TOKEN_UPDATE) {
                if (!m_have_token) {
                    m_have_token = true;
                    m_create_token_counter++;
                }
//                DEBUG_PRINT("eventTokenHandling %d\r\n", m_last_update_dt);
                m_next_token_holder_id = options->myID;
                reset_next_poll();
                m_last_update = xTaskGetTickCount();
                m_expect_reply = true;
                poll_next(dev);
            } else if (m_have_token && m_last_update_dt > RECEIVE_TIMEOUT && m_expect_reply &&
                       m_resend_counter < MAX_RESEND) {
                resend_msg(dev);
            } else if ((m_have_token && !m_expect_reply) || (m_have_token && m_resend_counter >= MAX_RESEND)) {
                if (m_next_poll_id < NUM_ACTIVE_DRONES || NUM_ACTIVE_DRONES <= 2) {
                    if (m_next_poll_id >= NUM_ACTIVE_DRONES) { // can only happen if we have 2 drones
                        reset_next_poll();
                    }
                    poll_next(dev);
                } else {//send token
                    pass_token(dev);
                }
            }
            return t_out;
//            DEBUG_PRINT("eventTokenHandling %d\r\n", m_last_update_dt);
//            break;
        default:
            configASSERT(false);
    }

    return MAX_TIMEOUT;
}

void reset_next_poll() {
    m_next_poll_id = (options->myID != 0) ? 0 : 1;
}
//
void reset_all_token_counter(){
    for (int i=0; i < NUM_ACTIVE_DRONES; i++){
        coll_dist_dev[i].token_resend_counter = 0;
    }
}

void pass_token(dwDevice_t *dev) {
    m_last_update = xTaskGetTickCount();
    int dist_dev = getDistanceDev(options->dw1000Addresses[m_next_token_holder_id]);

    while(m_next_token_holder_id == options->myID || coll_dist_dev[dist_dev].token_resend_counter >= MAX_RESEND){
        m_next_token_holder_id++;
        m_next_token_holder_id %= (NUM_ACTIVE_DRONES - 1);
        dist_dev = getDistanceDev(options->dw1000Addresses[m_next_token_holder_id]);
        if (options->myID > dist_dev && coll_dist_dev[dist_dev].poll_counter < MAX_RESEND)
            break;
        if (m_next_token_holder_id == options->myID) // if it looped, break loop
            break;
    }
    reset_all_token_counter();
    if (m_next_token_holder_id == options->myID){
//        m_next_token_holder_id++;
//        m_next_token_holder_id %= (NUM_ACTIVE_DRONES - 1);
        reset_next_poll();
        poll_next(dev);
    }else{
        m_last_update = xTaskGetTickCount();
        coll_dist_dev[dist_dev].poll_counter = 0;
        coll_dist_dev[dist_dev].token_resend_counter = 0;
        txPacket.sourceAddress = options->dw1000Addresses[options->myID];
        txPacket.destAddress = options->dw1000Addresses[m_next_token_holder_id];
        curr_seq++;
//        DEBUG_PRINT("pass_token %d poll_counter %d \r\n", (int)m_next_token_holder_id, coll_dist_dev[dist_dev].poll_counter);
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_PASS_TOKEN;
        txPacket.payload[LPS_TWR_SEQ] = curr_seq;
        m_resend_counter = 0;
        m_expect_reply = true;
        send_msg(dev, 0);
    }

}

static void initCollVoidDev(lpsAlgoOptions_t *algoOptions) {
    curr_seq = 0;
    for (int i = 0; i < NUM_ACTIVE_DRONES; i++) {
//        if (i == algoOptions->myID)
//            continue;
        coll_dist_dev[i].dev_address = algoOptions->dw1000Addresses[i];
        coll_dist_dev[i].ranging_complete = false;
        coll_dist_dev[i].poll_counter = 0;
        memset(&coll_dist_dev[i].poll_tx, 0, sizeof(coll_dist_dev[i].poll_tx));
        memset(&coll_dist_dev[i].poll_rx, 0, sizeof(coll_dist_dev[i].poll_rx));
        memset(&coll_dist_dev[i].answer_tx, 0, sizeof(coll_dist_dev[i].answer_tx));
        memset(&coll_dist_dev[i].answer_rx, 0, sizeof(coll_dist_dev[i].answer_rx));
        memset(&coll_dist_dev[i].final_tx, 0, sizeof(coll_dist_dev[i].final_tx));
        memset(&coll_dist_dev[i].final_rx, 0, sizeof(coll_dist_dev[i].final_rx));
    }
}

static void collvoidCommInit(dwDevice_t *dev, lpsAlgoOptions_t *algoOptions) {
    options = algoOptions;
//    DEBUG_PRINT("collvoidCommInit\r\n");
    // Initialize the packet in the TX buffer
    memset(&txPacket, 0, sizeof(txPacket));
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    txPacket.pan = 0xbccf;
    m_last_update = xTaskGetTickCount();
    curr_seq = 0;
    initCollVoidDev(algoOptions);

    options->rangingState = 0;
    ranging_complete = false;

    memset(options->distance, 0, sizeof(options->distance));
    memset(options->pressures, 0, sizeof(options->pressures));
    memset(options->failedRanging, 0, sizeof(options->failedRanging));
}

uwbAlgorithm_t collvoidCommAlgorithm = {
        .init = collvoidCommInit,
        .onEvent = collvoidCommOnEvent,
};

LOG_GROUP_START(twr)
LOG_ADD(LOG_FLOAT, rangingSuccessRate0, &rangingSuccessRate[0]
)
LOG_ADD(LOG_UINT32, rangingPerSec0, &rangingPerSec[0]
)
LOG_GROUP_STOP(twr)
