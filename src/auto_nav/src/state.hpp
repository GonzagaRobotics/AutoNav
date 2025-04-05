#pragma once

namespace FSM
{
    // Forward declaration
    class AutoNav;

    class State
    {
    public:
        virtual ~State() = default;
        virtual void enter(AutoNav *autoNav) = 0;
        virtual void exit(AutoNav *autoNav) = 0;
    };
}