// *************************************************************************
//
// Copyright (c) 2022 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include "canbus.hpp"
#include "TimerInterrupt.h"

#define RX_TASK_PRIO 1
#define TX_TASK_PRIO 2
#define CAN_CLAIM_MS 1000

CanBus devCanBus;

void InterruptTimerHandler(CanBus *dev_can)
{
    devCanBus.SendPresenceMsg();
}

void InterruptCanHandler()
{
    // TODO #3 too large for interruption content
    devCanBus.HandleCanRxInterrupt();
}

void CanBus::Start(uint8_t _CS, uint8_t address, uint8_t data_id)
{
    m_devAddr = address;

#if !DBG_DONT_INIT_CAN
    StartCan(_CS);
#endif

    InitInterrupts();
}

void CanBus::Send(can_frame *msg)
{
    if (m_canInit)
    {
        m_mcp2515.sendMessage(msg);
    }
}

void CanBus::Send(uint8_t data_id,
                  uint8_t bytes_0_8,
                  uint8_t d0,
                  uint8_t d1,
                  uint8_t d2,
                  uint8_t d3,
                  uint8_t d4,
                  uint8_t d5,
                  uint8_t d6,
                  uint8_t d7)
{
    if (m_canInit)
    {
        can_frame msg = {
            .can_id = GetCanId(m_devAddr, data_id),
            .can_dlc = bytes_0_8,
            .data = {d0, d1, d2, d3, d4, d5, d6, d7}};
        m_mcp2515.sendMessage(&msg);
    }
}

void CanBus::StartCan(uint8_t _CS)
{
    m_mcp2515.begin(_CS);
    m_mcp2515.reset();
    m_mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
    m_mcp2515.setNormalMode();
    m_canInit = true;
}

void CanBus::InitInterrupts()
{
    // can
    // https://github.com/an-dr/arduino-mcp2515#receive-data
    attachInterrupt(0, InterruptCanHandler, FALLING);

    // timer
    ITimer1.init();
    if (!ITimer1.attachInterruptInterval(CAN_CLAIM_MS, InterruptTimerHandler, this))
    {
        Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
    }
}

void CanBus::FillDescriptorPointers(const can_frame &msg,
                                    const CanBusAutomationDesctiptor &dsc)
{
    for (int i = 0; i < msg.can_dlc; ++i)
    {
        uint8_t *pDescrPointer = dsc.pData[i];
        if (pDescrPointer != 0)
        {
            *pDescrPointer = msg.data[i];
        }
    }
}

bool CanBus::SetDataToStore(uint32_t rx_msg_id,
                            uint8_t *d0_rx,
                            uint8_t *d1_rx,
                            uint8_t *d2_rx,
                            uint8_t *d3_rx,
                            uint8_t *d4_rx,
                            uint8_t *d5_rx,
                            uint8_t *d6_rx,
                            uint8_t *d7_rx)
{
    bool result = false;
    for (int i; i < RX_DESCRIPTORS_NUM; ++i)
    {
        if (m_rx_desc[i].msgId == 0) // not set
        {
            m_rx_desc[i].msgId = rx_msg_id;
            m_rx_desc[i].pData[0] = d0_rx;
            m_rx_desc[i].pData[1] = d1_rx;
            m_rx_desc[i].pData[2] = d2_rx;
            m_rx_desc[i].pData[3] = d3_rx;
            m_rx_desc[i].pData[4] = d4_rx;
            m_rx_desc[i].pData[5] = d5_rx;
            m_rx_desc[i].pData[6] = d6_rx;
            m_rx_desc[i].pData[7] = d7_rx;
            result = true;
            break;
        }
    }
    return result;
}

void CanBus::HandleCanRxInterrupt()
{
    uint8_t irq = m_mcp2515.getInterrupts();
    can_frame frame = {0};

    if (irq & MCP2515::CANINTF_RX0IF)
    {
        if (m_mcp2515.readMessage(MCP2515::RXB0, &frame) != MCP2515::ERROR_OK)
        {
            // TODO #3 remove prints from interrupt
            Serial.println("Error during RXB0 message processing");
        }
    }

    if (irq & MCP2515::CANINTF_RX1IF)
    {
        if (m_mcp2515.readMessage(MCP2515::RXB1, &frame) != MCP2515::ERROR_OK)
        {
            // TODO #3 remove prints from interrupt
            Serial.println("Error during RXB1 message processing");
        }
    }

    for (int i = 0; i < RX_DESCRIPTORS_NUM; ++i)
    {
        if (m_rx_desc[i].msgId == frame.can_id)
        {
            FillDescriptorPointers(frame, m_rx_desc[i]);
        }
    }

    if (m_pRxCallback != nullptr)
    {
        (*m_pRxCallback)(this, frame);
    }
}
