/*
 * io_expander.h
 *
 *  Created on: Dec 6, 2022
 *      Author: simonyu
 */

#ifndef PLATFORM_IO_EXPANDER_H_
#define PLATFORM_IO_EXPANDER_H_

#include <MCP23018.h>
#include <memory>
#include <vector>

namespace biped
{
class IOExpander
{
public:

    IOExpander(const uint8_t& address);

    std::shared_ptr<MCP23018>
    get() const;

    MCP23018*
    getRaw() const;

    void
    attachInterruptPortA(const uint8_t& pin, void
    (*handler)(void), const int& mode);

    void
    attachInterruptPortB(const uint8_t& pin, void
    (*handler)(void), const int& mode);

    void
    attachInterruptArgPortA(const uint8_t& pin, void
    (*handler)(void*), void* arg, const int& mode);

    void
    attachInterruptArgPortB(const uint8_t& pin, void
    (*handler)(void*), void* arg, const int& mode);

    void
    detachInterruptPortA(const uint8_t& pin);

    void
    detachInterruptPortB(const uint8_t& pin);

    void
    pinModePortA(const uint8_t& pin, const uint8_t& mode);

    void
    pinModePortB(const uint8_t& pin, const uint8_t& mode);

    bool
    digitalReadPortA(const uint8_t& pin);

    bool
    digitalReadPortB(const uint8_t& pin);

    void
    digitalWritePortA(const uint8_t& pin, const bool& state);

    void
    digitalWritePortB(const uint8_t& pin, const bool& state);

    void IRAM_ATTR
    onInterrupt();

private:

    struct InterruptHandler
    {
        void
        (*handler)(void);
        void
        (*handler_arg)(void*);
        void *arg;
        int mode;
        bool with_arg;

        InterruptHandler() : handler(nullptr), handler_arg(nullptr), arg(nullptr), mode(CHANGE),
                with_arg(false)
        {
        }
    };

    void
    handleInterrupt(const uint8_t& flags, const uint8_t& captures,
            const std::vector<InterruptHandler>& interrupt_handlers);

    std::vector<InterruptHandler> interrupt_handlers_port_a_;
    std::vector<InterruptHandler> interrupt_handlers_port_b_;
    std::shared_ptr<MCP23018> mcp23018_;
};
}

#endif
