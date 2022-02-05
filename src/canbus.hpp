// *************************************************************************
//
// Copyright (c) 2022 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include "mcp2515.h"

#define USE_TIMER_1 1
#define DBG_DONT_INIT_CAN 0
#define RX_DESCRIPTORS_NUM 1
#define TX_DESCRIPTORS_NUM 1

class CanBus; // Forward reference

typedef void (*CanBusCallback)(const CanBus *, const can_frame &);

struct CanBusAutomationDesctiptor
{
    uint32_t msgId;
    uint8_t *pData[CAN_MAX_DLEN];
    uint32_t periodTxMs;
} __attribute__((packed));

// TODO document methods
class CanBus
{
private:
    bool m_canInit;
    MCP2515 m_mcp2515;
    uint8_t m_devAddr;
    CanBusAutomationDesctiptor m_rx_desc[RX_DESCRIPTORS_NUM];
    CanBusAutomationDesctiptor m_tx_desc[TX_DESCRIPTORS_NUM];
    CanBusCallback m_pRxCallback;
    
    
    inline uint32_t GetCanId(uint8_t addr, uint8_t data_id)
    {
        return ((m_devAddr << 8) | data_id);
    };
    
    void StartCan(uint8_t _CS);
    void InitInterrupts();
    static void FillDescriptorPointers(const can_frame &msg,
                                       const CanBusAutomationDesctiptor &dsc);

public:
    CanBus() = default;

    // https://github.com/an-dr/zakhar/blob/master/docs/canbus.md#addressing
    void Start(uint8_t _CS, uint8_t address, uint8_t data_id = 0x1);

    void HandleCanRxInterrupt();

    // https://github.com/an-dr/zakhar/blob/master/docs/canbus.md#declare-the-presence
    inline void SendPresenceMsg() { Send(m_devAddr << 8); };

    void Send(uint8_t data_id,
              uint8_t bytes_0_8 = 0,
              uint8_t d0 = 0,
              uint8_t d1 = 0,
              uint8_t d2 = 0,
              uint8_t d3 = 0,
              uint8_t d4 = 0,
              uint8_t d5 = 0,
              uint8_t d6 = 0,
              uint8_t d7 = 0);
    void Send(can_frame *msg);

    inline void SetRxCallback(CanBusCallback callback_func) { m_pRxCallback = callback_func; };

    bool SetDataToStore(uint32_t rx_msg_id,
                        uint8_t *d0_rx = nullptr,
                        uint8_t *d1_rx = nullptr,
                        uint8_t *d2_rx = nullptr,
                        uint8_t *d3_rx = nullptr,
                        uint8_t *d4_rx = nullptr,
                        uint8_t *d5_rx = nullptr,
                        uint8_t *d6_rx = nullptr,
                        uint8_t *d7_rx = nullptr);

    // TODO implement periodic sending
    // void SetDataToSend(uint32_t period_ms,
    //                    uint32_t rx_msg_id,
    //                    uint8_t *d0_tx = nullptr,
    //                    uint8_t *d1_tx = nullptr,
    //                    uint8_t *d2_tx = nullptr,
    //                    uint8_t *d3_tx = nullptr,
    //                    uint8_t *d4_tx = nullptr,
    //                    uint8_t *d5_tx = nullptr,
    //                    uint8_t *d6_tx = nullptr,
    //                    uint8_t *d7_tx = nullptr);
};

extern CanBus devCanBus;
