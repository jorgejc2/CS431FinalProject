/*
 * io_expander.cpp
 *
 *  Created on: Dec 6, 2022
 *      Author: simonyu
 */

#include "common/global.h"
#include "platform/io_expander.h"
#include "common/parameter.h"
#include "platform/serial.h"

namespace biped
{
IOExpander::IOExpander(const uint8_t& address) :
        interrupt_handlers_port_a_(IOExpanderParameter::num_port_pins),
        interrupt_handlers_port_b_(IOExpanderParameter::num_port_pins)
{
    mcp23018_ = std::make_shared<MCP23018>(address);

    mcp23018_->begin();
    mcp23018_->writeToRegister(IOCON, IOCON_MIRROR | IOCON_INTPOL | IOCON_INTCC);
    mcp23018_->SetDirections(0xFF, 0xFF);
    mcp23018_->SetPullups(0xFF, 0xFF);
    mcp23018_->writePairToRegister(INTENA, 0x00, 0x00);
}

std::shared_ptr<MCP23018>
IOExpander::get() const
{
    return mcp23018_;
}

MCP23018*
IOExpander::getRaw() const
{
    return mcp23018_.get();
}

void
IOExpander::attachInterruptPortA(const uint8_t& pin, void
(*handler)(void), const int& mode)
{
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    interrupt_handlers_port_a_.at(pin).handler = handler;
    interrupt_handlers_port_a_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_a_.at(pin).arg = nullptr;
    interrupt_handlers_port_a_.at(pin).mode = mode;
    interrupt_handlers_port_a_.at(pin).with_arg = false;

    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    mcp23018_->setBitInRegister(INTENA, pin, true);
}

void
IOExpander::attachInterruptPortB(const uint8_t& pin, void
(*handler)(void), const int& mode)
{
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    interrupt_handlers_port_b_.at(pin).handler = handler;
    interrupt_handlers_port_b_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_b_.at(pin).arg = nullptr;
    interrupt_handlers_port_b_.at(pin).mode = mode;
    interrupt_handlers_port_b_.at(pin).with_arg = false;

    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    mcp23018_->setBitInRegister(INTENB, pin, true);
}

void
IOExpander::attachInterruptArgPortA(const uint8_t& pin, void
(*handler)(void*), void* arg, const int& mode)
{
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    interrupt_handlers_port_a_.at(pin).handler = nullptr;
    interrupt_handlers_port_a_.at(pin).handler_arg = handler;
    interrupt_handlers_port_a_.at(pin).arg = arg;
    interrupt_handlers_port_a_.at(pin).mode = mode;
    interrupt_handlers_port_a_.at(pin).with_arg = true;

    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    mcp23018_->setBitInRegister(INTENA, pin, true);
}

void
IOExpander::attachInterruptArgPortB(const uint8_t& pin, void
(*handler)(void*), void* arg, const int& mode)
{
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    interrupt_handlers_port_b_.at(pin).handler = nullptr;
    interrupt_handlers_port_b_.at(pin).handler_arg = handler;
    interrupt_handlers_port_b_.at(pin).arg = arg;
    interrupt_handlers_port_b_.at(pin).mode = mode;
    interrupt_handlers_port_b_.at(pin).with_arg = true;

    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    mcp23018_->setBitInRegister(INTENB, pin, true);
}

void
IOExpander::detachInterruptPortA(const uint8_t& pin)
{
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    interrupt_handlers_port_a_.at(pin).handler = nullptr;
    interrupt_handlers_port_a_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_a_.at(pin).arg = nullptr;
    interrupt_handlers_port_a_.at(pin).mode = ONLOW;
    interrupt_handlers_port_a_.at(pin).with_arg = false;

    mcp23018_->setBitInRegister(INTENA, pin, false);
}

void
IOExpander::detachInterruptPortB(const uint8_t& pin)
{
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    interrupt_handlers_port_b_.at(pin).handler = nullptr;
    interrupt_handlers_port_b_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_b_.at(pin).arg = nullptr;
    interrupt_handlers_port_b_.at(pin).mode = ONLOW;
    interrupt_handlers_port_b_.at(pin).with_arg = false;

    mcp23018_->setBitInRegister(INTENB, pin, false);
}

void
IOExpander::pinModePortA(const uint8_t& pin, const uint8_t& mode)
{
    switch (mode)
    {
        case INPUT:
        {
            mcp23018_->setBitInRegister(IODIRA, pin, true);
            mcp23018_->setBitInRegister(GPPUA, pin, false);
            break;
        }
        case INPUT_PULLUP:
        {
            mcp23018_->setBitInRegister(IODIRA, pin, true);
            mcp23018_->setBitInRegister(GPPUA, pin, true);
            break;
        }
        case OUTPUT:
        {
            mcp23018_->setBitInRegister(IODIRA, pin, false);
            mcp23018_->setBitInRegister(GPPUA, pin, true);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unknown pin mode.";
            break;
        }
    }
}

void
IOExpander::pinModePortB(const uint8_t& pin, const uint8_t& mode)
{
    switch (mode)
    {
        case INPUT:
        {
            mcp23018_->setBitInRegister(IODIRB, pin, true);
            mcp23018_->setBitInRegister(GPPUB, pin, false);
            break;
        }
        case INPUT_PULLUP:
        {
            mcp23018_->setBitInRegister(IODIRB, pin, true);
            mcp23018_->setBitInRegister(GPPUB, pin, true);
            break;
        }
        case OUTPUT:
        {
            mcp23018_->setBitInRegister(IODIRB, pin, false);
            mcp23018_->setBitInRegister(GPPUB, pin, true);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unknown pin mode.";
            break;
        }
    }
}

bool
IOExpander::digitalReadPortA(const uint8_t& pin)
{
    const int ret = mcp23018_->GetPortA();
    if (ret < 0) {
        Serial(LogLevel::error) << "read port A" << (int) pin << ": " << ret;
        return false;
    }
    const uint8_t pins = static_cast<uint8_t>(ret);
    return static_cast<bool>(pins & (1 << pin));
}

bool
IOExpander::digitalReadPortB(const uint8_t& pin)
{
    const int ret = mcp23018_->GetPortB();
    if (ret < 0) {
        Serial(LogLevel::error) << "read port B" << (int) pin << ": " << ret;
        return false;
    }
    const uint8_t pins = static_cast<uint8_t>(ret);
    return static_cast<bool>(pins & (1 << pin));
}

void
IOExpander::digitalWritePortA(const uint8_t& pin, const bool& state)
{
    mcp23018_->SetAPin(pin, state);
}

void
IOExpander::digitalWritePortB(const uint8_t& pin, const bool& state)
{
    mcp23018_->SetBPin(pin, state);
}

void IRAM_ATTR
IOExpander::onInterrupt()
{
    lock_wire_.lock();
    const int interrupt_flags = mcp23018_->readPairFromRegister(INTFA);
    if (interrupt_flags < 0) {
        Serial(LogLevel::error) << "read int flag AB: " << interrupt_flags;
        return;
    }
    const int interrupt_captures = mcp23018_->readPairFromRegister(INTCAPA);
    if (interrupt_captures < 0) {
        Serial(LogLevel::error) << "read int cap AB: " << interrupt_captures;
        return;
    }
    lock_wire_.unlock();

    const uint8_t interrupt_flags_port_a = static_cast<uint8_t>(interrupt_flags);
    const uint8_t interrupt_flags_port_b = static_cast<uint8_t>(interrupt_flags
            >> IOExpanderParameter::num_port_pins);
    const uint8_t interrupt_captures_port_a = static_cast<uint8_t>(interrupt_captures);
    const uint8_t interrupt_captures_port_b = static_cast<uint8_t>(interrupt_captures
            >> IOExpanderParameter::num_port_pins);

    handleInterrupt(interrupt_flags_port_a, interrupt_captures_port_a, interrupt_handlers_port_a_);
    handleInterrupt(interrupt_flags_port_b, interrupt_captures_port_b, interrupt_handlers_port_b_);
}

void
IOExpander::handleInterrupt(const uint8_t& flags, const uint8_t& captures,
        const std::vector<InterruptHandler>& interrupt_handlers)
{
    uint8_t mask = 1;

    for (uint8_t pin = 0; pin < IOExpanderParameter::num_port_pins; pin ++, mask <<= 1)
    {
        if (flags & mask)
        {
            if (interrupt_handlers.at(pin).with_arg)
            {
                if (interrupt_handlers.at(pin).handler_arg == nullptr)
                {
                    continue;
                }
            }
            else
            {
                if (interrupt_handlers.at(pin).handler == nullptr)
                {
                    continue;
                }
            }

            if (interrupt_handlers.at(pin).mode == RISING)
            {
                if (captures & mask)
                {
                    if (interrupt_handlers.at(pin).with_arg)
                    {
                        interrupt_handlers.at(pin).handler_arg(interrupt_handlers.at(pin).arg);
                    }
                    else
                    {
                        interrupt_handlers.at(pin).handler();
                    }
                }
            }
            else if (interrupt_handlers.at(pin).mode == FALLING)
            {
                if (!(captures & mask))
                {
                    if (interrupt_handlers.at(pin).with_arg)
                    {
                        interrupt_handlers.at(pin).handler_arg(interrupt_handlers.at(pin).arg);
                    }
                    else
                    {
                        interrupt_handlers.at(pin).handler();
                    }
                }
            }
            else
            {
                if (interrupt_handlers.at(pin).with_arg)
                {
                    interrupt_handlers.at(pin).handler_arg(interrupt_handlers.at(pin).arg);
                }
                else
                {
                    interrupt_handlers.at(pin).handler();
                }
            }
        }
    }
}
}
